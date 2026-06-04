#include "amr_visualization/robot_open_gl_widget.hpp"

#include <QDataStream>
#include <QFile>
#include <QFileInfo>
#include <QOpenGLContext>
#include <QOpenGLShader>
#include <QSurfaceFormat>

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr::visualization
{

namespace
{

constexpr double k_pi = 3.14159265358979323846;

bool finite_vector(const QVector3D &point)
{
  return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}

QVector3D scaled_mesh_point(const RobotVisual &visual, const QVector3D &point)
{
  const double unit_scale = visual.mesh_auto_unit_scale ? visual.mesh_unit_scale : 1.0;
  return QVector3D(
    static_cast<float>(point.x() * visual.mesh_scale_x * unit_scale),
    static_cast<float>(point.y() * visual.mesh_scale_y * unit_scale),
    static_cast<float>(point.z() * visual.mesh_scale_z * unit_scale));
}

void append_triangle(
  const QVector3D &a,
  const QVector3D &b,
  const QVector3D &c,
  QVector<float> &vertices,
  QVector<unsigned int> &indices)
{
  QVector3D normal = QVector3D::crossProduct(b - a, c - a);
  if (normal.lengthSquared() < 1e-12F) {
    normal = QVector3D(0.0F, 0.0F, 1.0F);
  } else {
    normal.normalize();
  }

  const unsigned int base_index = static_cast<unsigned int>(vertices.size() / 6);
  const QVector3D points[3] = {a, b, c};
  for (const auto &point : points) {
    vertices.push_back(point.x());
    vertices.push_back(point.y());
    vertices.push_back(point.z());
    vertices.push_back(normal.x());
    vertices.push_back(normal.y());
    vertices.push_back(normal.z());
  }
  indices.push_back(base_index);
  indices.push_back(base_index + 1U);
  indices.push_back(base_index + 2U);
}

}  // namespace

RobotOpenGLWidget::RobotOpenGLWidget(QWidget *parent)
: QOpenGLWidget(parent)
{
  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(0);
  format.setAlphaBufferSize(8);
  format.setSamples(4);
  setFormat(format);
  setAutoFillBackground(false);
  setAttribute(Qt::WA_AlwaysStackOnTop, true);
  setAttribute(Qt::WA_TransparentForMouseEvents, true);
  setAttribute(Qt::WA_TranslucentBackground, true);
  hide();
}

RobotOpenGLWidget::~RobotOpenGLWidget()
{
  if (initialized_ && context()) {
    makeCurrent();
    destroyAllMeshBuffers();
    program_.removeAllShaders();
    doneCurrent();
  }
}

void RobotOpenGLWidget::setRobotVisuals(const QVector<RobotVisual> &visuals)
{
  QElapsedTimer timer;
  timer.start();
  visuals_.clear();
  QSet<QString> active_paths;
  for (const auto &visual : visuals) {
    if (
      visual.valid &&
      visual.type == RobotGeometryType::Mesh &&
      visual.mesh_enabled &&
      visual.mesh_render_mode == "opengl" &&
      !visual.mesh_resolved_path.isEmpty())
    {
      visuals_.push_back(visual);
      active_paths.insert(visual.mesh_resolved_path);
    }
  }

  pruneInactiveMeshes(active_paths);
  for (const auto &visual : visuals_) {
    if (mesh_cache_.find(visual.mesh_resolved_path) != mesh_cache_.end()) {
      continue;
    }

    auto mesh = std::make_unique<GpuMesh>();
    mesh->attempted = true;
    loadStlMesh(visual, *mesh);
    if (mesh->rejected || mesh->vertices.isEmpty() || mesh->indices.isEmpty()) {
      if (!warning_cache_.contains(visual.mesh_resolved_path)) {
        warning_cache_.insert(visual.mesh_resolved_path);
        Q_EMIT visualizationEvent(
          QString("OpenGL robot mesh load failed for %1: %2")
            .arg(visual.mesh_filename, mesh->error));
      }
    } else if (!success_cache_.contains(visual.mesh_resolved_path)) {
      success_cache_.insert(visual.mesh_resolved_path);
      Q_EMIT visualizationEvent(
        QString("OpenGL robot mesh ready: %1 (%2 triangle(s), bbox=%3, frame=%4)")
          .arg(visual.mesh_filename)
          .arg(mesh->indices.size() / 3)
          .arg(meshBoundsText(*mesh))
          .arg(visual.frame_id));
    }
    mesh_cache_[visual.mesh_resolved_path] = std::move(mesh);
  }

  if (timer.elapsed() > 33) {
    Q_EMIT visualizationEvent(
      QString("OpenGL robot mesh update slow: %1 ms").arg(timer.elapsed()));
  }
  setVisible(hasRenderableVisuals());
  update();
}

void RobotOpenGLWidget::setCamera(
  const QVector3D &focal_point,
  const double yaw,
  const double pitch,
  const double distance,
  const double pixels_per_meter)
{
  focal_point_ = focal_point;
  camera_yaw_ = yaw;
  camera_pitch_ = pitch;
  camera_distance_ = distance;
  pixels_per_meter_ = std::max(pixels_per_meter, 1.0);
  if (isVisible()) {
    update();
  }
}

bool RobotOpenGLWidget::hasRenderableMesh(const RobotVisual &visual) const
{
  if (
    visual.type != RobotGeometryType::Mesh ||
    !visual.mesh_enabled ||
    visual.mesh_render_mode != "opengl" ||
    visual.mesh_resolved_path.isEmpty())
  {
    return false;
  }
  const auto it = mesh_cache_.find(visual.mesh_resolved_path);
  return it != mesh_cache_.end() && it->second && !it->second->rejected &&
    it->second->uploaded && initialized_ && program_.isLinked();
}

bool RobotOpenGLWidget::hasRenderableVisuals() const
{
  for (const auto &visual : visuals_) {
    const auto it = mesh_cache_.find(visual.mesh_resolved_path);
    if (
      it != mesh_cache_.end() &&
      it->second &&
      !it->second->rejected &&
      !it->second->vertices.isEmpty() &&
      !it->second->indices.isEmpty())
    {
      return true;
    }
  }
  return false;
}

void RobotOpenGLWidget::initializeGL()
{
  initializeOpenGLFunctions();
  initialized_ = true;
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClearColor(0.0F, 0.0F, 0.0F, 0.0F);

  const char *vertex_shader =
    "attribute vec3 position;\n"
    "attribute vec3 normal;\n"
    "uniform mat4 mvp;\n"
    "uniform mat3 normal_matrix;\n"
    "varying vec3 v_normal;\n"
    "void main() {\n"
    "  v_normal = normalize(normal_matrix * normal);\n"
    "  gl_Position = mvp * vec4(position, 1.0);\n"
    "}\n";
  const char *fragment_shader =
    "#ifdef GL_ES\n"
    "precision mediump float;\n"
    "#endif\n"
    "varying vec3 v_normal;\n"
    "uniform vec3 light_dir;\n"
    "void main() {\n"
    "  float diffuse = abs(dot(normalize(v_normal), normalize(light_dir)));\n"
    "  float shade = 0.34 + (diffuse * 0.54);\n"
    "  gl_FragColor = vec4(vec3(shade), 0.96);\n"
    "}\n";

  if (
    !program_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader) ||
    !program_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader) ||
    !program_.link())
  {
    Q_EMIT visualizationEvent(QString("OpenGL robot shader setup failed: %1").arg(program_.log()));
  } else {
    Q_EMIT visualizationEvent("OpenGL robot renderer initialized with depth testing");
  }
}

void RobotOpenGLWidget::resizeGL(int, int)
{
  update();
}

void RobotOpenGLWidget::paintGL()
{
  glViewport(0, 0, width(), height());
  glClearColor(0.0F, 0.0F, 0.0F, 0.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (!program_.isLinked()) {
    return;
  }

  const QMatrix4x4 view = viewMatrix();
  const QMatrix4x4 projection = projectionMatrix();
  const QVector3D light_dir =
    (cameraUp() - cameraForward() + QVector3D(0.25F, 0.20F, 0.35F)).normalized();

  program_.bind();
  program_.setUniformValue("light_dir", light_dir);
  for (const auto &visual : visuals_) {
    auto it = mesh_cache_.find(visual.mesh_resolved_path);
    if (it == mesh_cache_.end() || !it->second || it->second->rejected) {
      continue;
    }
    GpuMesh &mesh = *it->second;
    bool uploaded_now = false;
    if (!mesh.uploaded) {
      if (!uploadMesh(mesh)) {
        if (parentWidget()) {
          parentWidget()->update();
        }
        continue;
      }
      uploaded_now = true;
    }
    if (!mesh.vao.isCreated() || mesh.indices.isEmpty()) {
      continue;
    }

    const QMatrix4x4 model = modelMatrixForVisual(visual);
    program_.setUniformValue("mvp", projection * view * model);
    program_.setUniformValue("normal_matrix", model.normalMatrix());
    QOpenGLVertexArrayObject::Binder vao_binder(&mesh.vao);
    glDrawElements(GL_TRIANGLES, mesh.indices.size(), GL_UNSIGNED_INT, nullptr);
    if (uploaded_now && parentWidget()) {
      parentWidget()->update();
    }

    const QString render_key = visual.mesh_resolved_path;
    if (!render_event_cache_.contains(render_key)) {
      render_event_cache_.insert(render_key);
      Q_EMIT visualizationEvent(
        QString("OpenGL robot mesh rendered: %1 (%2 triangle(s))")
          .arg(visual.mesh_filename)
          .arg(mesh.indices.size() / 3));
    }
  }
  program_.release();
}

QVector3D RobotOpenGLWidget::cameraRight() const
{
  return QVector3D(
    static_cast<float>(std::cos(camera_yaw_)),
    static_cast<float>(std::sin(camera_yaw_)),
    0.0F).normalized();
}

QVector3D RobotOpenGLWidget::cameraForward() const
{
  const QVector3D ground_up(
    static_cast<float>(-std::sin(camera_yaw_)),
    static_cast<float>(std::cos(camera_yaw_)),
    0.0F);
  return QVector3D(
    ground_up.x() * static_cast<float>(std::cos(camera_pitch_)),
    ground_up.y() * static_cast<float>(std::cos(camera_pitch_)),
    static_cast<float>(-std::sin(camera_pitch_))).normalized();
}

QVector3D RobotOpenGLWidget::cameraUp() const
{
  return QVector3D::crossProduct(cameraRight(), cameraForward()).normalized();
}

QMatrix4x4 RobotOpenGLWidget::modelMatrixForVisual(const RobotVisual &visual) const
{
  QMatrix4x4 model;
  model.translate(
    static_cast<float>(visual.pose.x),
    static_cast<float>(visual.pose.y),
    static_cast<float>(visual.pose.z));
  model.rotate(static_cast<float>(visual.pose.yaw * 180.0 / k_pi), 0.0F, 0.0F, 1.0F);
  model.rotate(static_cast<float>(visual.pose.pitch * 180.0 / k_pi), 0.0F, 1.0F, 0.0F);
  model.rotate(static_cast<float>(visual.pose.roll * 180.0 / k_pi), 1.0F, 0.0F, 0.0F);
  const double unit_scale = visual.mesh_auto_unit_scale ? visual.mesh_unit_scale : 1.0;
  model.scale(
    static_cast<float>(visual.mesh_scale_x * unit_scale),
    static_cast<float>(visual.mesh_scale_y * unit_scale),
    static_cast<float>(visual.mesh_scale_z * unit_scale));
  return model;
}

QMatrix4x4 RobotOpenGLWidget::viewMatrix() const
{
  QMatrix4x4 view;
  const QVector3D eye =
    focal_point_ - (cameraForward() * static_cast<float>(std::max(camera_distance_, 0.1)));
  view.lookAt(eye, focal_point_, cameraUp());
  return view;
}

QMatrix4x4 RobotOpenGLWidget::projectionMatrix() const
{
  QMatrix4x4 projection;
  const float half_width = static_cast<float>(width() / (2.0 * std::max(pixels_per_meter_, 1.0)));
  const float half_height = static_cast<float>(height() / (2.0 * std::max(pixels_per_meter_, 1.0)));
  const float far_plane = static_cast<float>(std::max(camera_distance_ + 100.0, 120.0));
  projection.ortho(-half_width, half_width, -half_height, half_height, 0.01F, far_plane);
  return projection;
}

bool RobotOpenGLWidget::loadStlMesh(const RobotVisual &visual, GpuMesh &mesh)
{
  const QString path = visual.mesh_resolved_path;
  const QFileInfo file_info(path);
  if (file_info.suffix().compare("stl", Qt::CaseInsensitive) != 0) {
    mesh.rejected = true;
    mesh.error = QString("unsupported extension .%1").arg(file_info.suffix());
    return false;
  }

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    mesh.rejected = true;
    mesh.error = file.errorString();
    return false;
  }

  const qint64 file_size = file.size();
  const qint64 max_file_size_bytes =
    static_cast<qint64>(std::max(visual.mesh_max_file_size_mb, 1)) * 1024LL * 1024LL;
  if (file_size <= 0) {
    mesh.rejected = true;
    mesh.error = "empty STL file";
    return false;
  }
  if (file_size > max_file_size_bytes) {
    mesh.rejected = true;
    mesh.error = QString("STL file exceeds %1 MiB limit").arg(visual.mesh_max_file_size_mb);
    return false;
  }

  if (file_size >= 84) {
    file.seek(80);
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    quint32 triangle_count = 0;
    stream >> triangle_count;
    const qint64 expected_size = 84 + (static_cast<qint64>(triangle_count) * 50);
    if (triangle_count > 0 && expected_size == file_size) {
      const quint32 max_loaded_triangles =
        static_cast<quint32>(std::max(visual.mesh_max_loaded_triangles, 1));
      const quint32 loaded_count = std::min(triangle_count, max_loaded_triangles);
      mesh.vertices.reserve(static_cast<int>(loaded_count * 18U));
      mesh.indices.reserve(static_cast<int>(loaded_count * 3U));
      for (quint32 i = 0; i < loaded_count && !stream.atEnd(); ++i) {
        float nx = 0.0F;
        float ny = 0.0F;
        float nz = 0.0F;
        float ax = 0.0F;
        float ay = 0.0F;
        float az = 0.0F;
        float bx = 0.0F;
        float by = 0.0F;
        float bz = 0.0F;
        float cx = 0.0F;
        float cy = 0.0F;
        float cz = 0.0F;
        quint16 attribute = 0;
        stream >> nx >> ny >> nz >> ax >> ay >> az >> bx >> by >> bz >> cx >> cy >> cz >> attribute;
        (void)nx;
        (void)ny;
        (void)nz;
        append_triangle(
          QVector3D(ax, ay, az),
          QVector3D(bx, by, bz),
          QVector3D(cx, cy, cz),
          mesh.vertices,
          mesh.indices);
      }
      if (triangle_count > loaded_count) {
        Q_EMIT visualizationEvent(
          QString("OpenGL robot mesh triangle load capped: loaded %1 of %2")
            .arg(loaded_count)
            .arg(triangle_count));
      }
      return validateMesh(visual, mesh);
    }
  }

  file.seek(0);
  const QByteArray header = file.peek(512).trimmed();
  if (!header.startsWith("solid")) {
    mesh.rejected = true;
    mesh.error = "not a recognized binary or ASCII STL";
    return false;
  }

  const QString content = QString::fromLatin1(file.readAll());
  QVector<QVector3D> triangle_vertices;
  triangle_vertices.reserve(3);
  quint32 parsed_triangles = 0;
  const quint32 max_loaded_triangles =
    static_cast<quint32>(std::max(visual.mesh_max_loaded_triangles, 1));
  const QStringList lines = content.split('\n');
  for (const QString &line : lines) {
    if (parsed_triangles >= max_loaded_triangles) {
      break;
    }
    const QString normalized = line.simplified();
    if (!normalized.startsWith("vertex ")) {
      continue;
    }
    const QStringList tokens = normalized.split(' ', Qt::SkipEmptyParts);
    if (tokens.size() != 4) {
      continue;
    }
    bool ok_x = false;
    bool ok_y = false;
    bool ok_z = false;
    const float x = tokens[1].toFloat(&ok_x);
    const float y = tokens[2].toFloat(&ok_y);
    const float z = tokens[3].toFloat(&ok_z);
    if (!ok_x || !ok_y || !ok_z) {
      continue;
    }
    triangle_vertices.push_back(QVector3D(x, y, z));
    if (triangle_vertices.size() == 3) {
      append_triangle(
        triangle_vertices[0],
        triangle_vertices[1],
        triangle_vertices[2],
        mesh.vertices,
        mesh.indices);
      triangle_vertices.clear();
      ++parsed_triangles;
    }
  }

  if (mesh.indices.isEmpty()) {
    mesh.rejected = true;
    mesh.error = "not a supported ASCII or binary STL";
    return false;
  }
  if (parsed_triangles >= max_loaded_triangles) {
    Q_EMIT visualizationEvent(
      QString("OpenGL robot mesh triangle load capped: loaded first %1 ASCII triangle(s)")
        .arg(max_loaded_triangles));
  }
  return validateMesh(visual, mesh);
}

bool RobotOpenGLWidget::validateMesh(const RobotVisual &visual, GpuMesh &mesh)
{
  if (mesh.vertices.isEmpty() || mesh.indices.isEmpty()) {
    mesh.rejected = true;
    mesh.error = "triangle count is zero";
    return false;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  bool finite = true;
  bool coordinate_in_range = true;
  const float max_abs_coordinate =
    static_cast<float>(std::max(visual.mesh_max_abs_coordinate_m, 0.001));

  for (int i = 0; i + 2 < mesh.vertices.size(); i += 6) {
    const QVector3D raw_point(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]);
    const QVector3D point = scaled_mesh_point(visual, raw_point);
    if (!finite_vector(point)) {
      finite = false;
      break;
    }
    if (
      std::abs(point.x()) > max_abs_coordinate ||
      std::abs(point.y()) > max_abs_coordinate ||
      std::abs(point.z()) > max_abs_coordinate)
    {
      coordinate_in_range = false;
    }
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    min_z = std::min(min_z, point.z());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
    max_z = std::max(max_z, point.z());
  }

  mesh.min_bounds = QVector3D(min_x, min_y, min_z);
  mesh.max_bounds = QVector3D(max_x, max_y, max_z);
  mesh.extent = mesh.max_bounds - mesh.min_bounds;
  mesh.diagonal = static_cast<double>(mesh.extent.length());
  if (!finite) {
    mesh.rejected = true;
    mesh.error = "non-finite coordinate";
  } else if (!coordinate_in_range) {
    mesh.rejected = true;
    mesh.error =
      QString("coordinate exceeds %1 m absolute limit").arg(visual.mesh_max_abs_coordinate_m);
  } else if (!std::isfinite(mesh.diagonal) || mesh.diagonal <= 0.0) {
    mesh.rejected = true;
    mesh.error = "invalid bbox diagonal";
  } else if (mesh.diagonal > visual.mesh_max_extent_m) {
    mesh.rejected = true;
    mesh.error =
      QString("bbox diagonal %1 m exceeds %2 m").arg(mesh.diagonal).arg(visual.mesh_max_extent_m);
  }

  if (mesh.rejected) {
    mesh.vertices.clear();
    mesh.indices.clear();
    return false;
  }
  return true;
}

bool RobotOpenGLWidget::uploadMesh(GpuMesh &mesh)
{
  if (!initialized_ || mesh.vertices.isEmpty() || mesh.indices.isEmpty()) {
    return false;
  }
  if (!program_.isLinked()) {
    mesh.rejected = true;
    mesh.error = "shader program is not linked";
    Q_EMIT visualizationEvent(QString("OpenGL robot mesh upload failed: %1").arg(mesh.error));
    return false;
  }
  if (!mesh.vertex_buffer.create() || !mesh.index_buffer.create() || !mesh.vao.create()) {
    mesh.rejected = true;
    mesh.error = "failed to create OpenGL buffers";
    destroyMeshBuffers(mesh);
    Q_EMIT visualizationEvent(QString("OpenGL robot mesh upload failed: %1").arg(mesh.error));
    return false;
  }

  program_.bind();
  QOpenGLVertexArrayObject::Binder vao_binder(&mesh.vao);
  mesh.vertex_buffer.bind();
  mesh.vertex_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  mesh.vertex_buffer.allocate(
    mesh.vertices.constData(),
    mesh.vertices.size() * static_cast<int>(sizeof(float)));
  mesh.index_buffer.bind();
  mesh.index_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
  mesh.index_buffer.allocate(
    mesh.indices.constData(),
    mesh.indices.size() * static_cast<int>(sizeof(unsigned int)));

  const int position_location = program_.attributeLocation("position");
  const int normal_location = program_.attributeLocation("normal");
  program_.enableAttributeArray(position_location);
  program_.setAttributeBuffer(
    position_location,
    GL_FLOAT,
    0,
    3,
    6 * static_cast<int>(sizeof(float)));
  program_.enableAttributeArray(normal_location);
  program_.setAttributeBuffer(
    normal_location,
    GL_FLOAT,
    3 * static_cast<int>(sizeof(float)),
    3,
    6 * static_cast<int>(sizeof(float)));
  mesh.vertex_buffer.release();
  mesh.index_buffer.release();
  program_.release();
  mesh.uploaded = true;
  return true;
}

void RobotOpenGLWidget::pruneInactiveMeshes(const QSet<QString> &active_paths)
{
  const bool can_destroy_gl = initialized_ && context();
  if (can_destroy_gl) {
    makeCurrent();
  }
  for (auto it = mesh_cache_.begin(); it != mesh_cache_.end();) {
    if (active_paths.contains(it->first)) {
      ++it;
      continue;
    }
    warning_cache_.remove(it->first);
    success_cache_.remove(it->first);
    render_event_cache_.remove(it->first);
    if (can_destroy_gl && it->second) {
      destroyMeshBuffers(*it->second);
    }
    it = mesh_cache_.erase(it);
  }
  if (can_destroy_gl) {
    doneCurrent();
  }
}

void RobotOpenGLWidget::destroyMeshBuffers(GpuMesh &mesh)
{
  if (mesh.vao.isCreated()) {
    mesh.vao.destroy();
  }
  if (mesh.vertex_buffer.isCreated()) {
    mesh.vertex_buffer.destroy();
  }
  if (mesh.index_buffer.isCreated()) {
    mesh.index_buffer.destroy();
  }
  mesh.uploaded = false;
}

void RobotOpenGLWidget::destroyAllMeshBuffers()
{
  for (auto &entry : mesh_cache_) {
    if (entry.second) {
      destroyMeshBuffers(*entry.second);
    }
  }
}

QString RobotOpenGLWidget::meshBoundsText(const GpuMesh &mesh) const
{
  return QString("min=(%1,%2,%3), max=(%4,%5,%6), diag=%7")
    .arg(mesh.min_bounds.x())
    .arg(mesh.min_bounds.y())
    .arg(mesh.min_bounds.z())
    .arg(mesh.max_bounds.x())
    .arg(mesh.max_bounds.y())
    .arg(mesh.max_bounds.z())
    .arg(mesh.diagonal);
}

}  // namespace amr::visualization

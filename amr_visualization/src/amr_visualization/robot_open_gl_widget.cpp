#include "amr_visualization/robot_open_gl_widget.hpp"

#include "amr_visualization/file_probe.hpp"

#include <QDataStream>
#include <QFile>
#include <QFileInfo>
#include <QOpenGLContext>
#include <QOpenGLShader>
#include <QLoggingCategory>
#include <QSurfaceFormat>
#include <QStringList>
#include <QVector4D>

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr::visualization
{

namespace
{

constexpr double k_pi = 3.14159265358979323846;

Q_LOGGING_CATEGORY(amrVizOpenGLLog, "amr_visualization.opengl")
Q_LOGGING_CATEGORY(amrVizMeshLog, "amr_visualization.mesh")

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

QString utf8_hex_dump(const QString &text)
{
  const QByteArray bytes = text.toUtf8();
  QStringList parts;
  parts.reserve(bytes.size());
  for (const char byte : bytes) {
    parts.push_back(QString("%1").arg(static_cast<unsigned char>(byte), 2, 16, QLatin1Char('0')));
  }
  return parts.join(' ');
}

QString escaped_path_text(const QString &text)
{
  QString escaped;
  escaped.reserve(text.size());
  for (const QChar character : text) {
    if (character == '\\') {
      escaped += "\\\\";
    } else if (character == '\n') {
      escaped += "\\n";
    } else if (character == '\r') {
      escaped += "\\r";
    } else if (character == '\t') {
      escaped += "\\t";
    } else if (character == '\'') {
      escaped += "\\'";
    } else if (character.unicode() < 0x20) {
      escaped += QString("\\u%1").arg(character.unicode(), 4, 16, QLatin1Char('0'));
    } else {
      escaped += character;
    }
  }
  return escaped;
}

QString file_access_diagnostics_text(const QString &uri, const QString &resolved_path)
{
  const FileProbe probe = probeFilePath(uri, resolved_path);
  return fileProbeToDiagnosticText(probe);
}

bool is_real_mesh_uri_or_path(const QString &mesh_filename)
{
  const QString trimmed = mesh_filename.trimmed();
  if (trimmed.isEmpty()) {
    return false;
  }
  if (trimmed.startsWith("package://") || trimmed.startsWith("file://")) {
    return true;
  }
  const QFileInfo file_info(trimmed);
  return file_info.isAbsolute() || file_info.suffix().compare("stl", Qt::CaseInsensitive) == 0;
}

bool is_urdf_mesh_visual(const RobotVisual &visual)
{
  return visual.valid && visual.type == RobotGeometryType::Mesh &&
    !visual.mesh_filename.trimmed().isEmpty() && is_real_mesh_uri_or_path(visual.mesh_filename) &&
    !visual.proxy_visual;
}

QString opengl_candidate_skip_reason(const RobotVisual &visual)
{
  if (!visual.valid) {
    return "invalid visual";
  }
  if (visual.type != RobotGeometryType::Mesh) {
    return visual.proxy_visual ? "proxy visual" : "non-mesh visual";
  }
  if (visual.proxy_visual) {
    return "proxy visual";
  }
  if (visual.mesh_filename.trimmed().isEmpty()) {
    return "empty mesh filename";
  }
  if (!is_real_mesh_uri_or_path(visual.mesh_filename)) {
    return "mesh filename is not a real URI/path";
  }
  if (visual.mesh_resolved_path.trimmed().isEmpty()) {
    return "unresolved mesh path";
  }
  if (!visual.mesh_enabled) {
    return "mesh rendering disabled";
  }
  if (visual.mesh_render_mode != "opengl") {
    return QString("mesh render mode is %1").arg(visual.mesh_render_mode);
  }

  const FileProbe probe = probeFilePath(visual.mesh_filename, visual.mesh_resolved_path);
  if (!probe.exists) {
    return "local mesh file does not exist";
  }
  if (!probe.is_file) {
    return "local mesh path is not a file";
  }
  if (!probe.readable) {
    return "local mesh file is not readable";
  }
  if (!probe.open_ok) {
    return QString("local mesh file open failed: %1").arg(probe.error_string);
  }
  return {};
}

bool is_accepted_opengl_mesh_visual(const RobotVisual &visual)
{
  return opengl_candidate_skip_reason(visual).isEmpty();
}

QString vector_text(const QVector3D &point)
{
  return QString("(%1,%2,%3)").arg(point.x()).arg(point.y()).arg(point.z());
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

void append_box(QVector<float> &vertices, QVector<unsigned int> &indices)
{
  const QVector3D p000(-0.5F, -0.5F, -0.5F);
  const QVector3D p001(-0.5F, -0.5F, 0.5F);
  const QVector3D p010(-0.5F, 0.5F, -0.5F);
  const QVector3D p011(-0.5F, 0.5F, 0.5F);
  const QVector3D p100(0.5F, -0.5F, -0.5F);
  const QVector3D p101(0.5F, -0.5F, 0.5F);
  const QVector3D p110(0.5F, 0.5F, -0.5F);
  const QVector3D p111(0.5F, 0.5F, 0.5F);

  append_triangle(p000, p100, p110, vertices, indices);
  append_triangle(p000, p110, p010, vertices, indices);
  append_triangle(p100, p101, p111, vertices, indices);
  append_triangle(p100, p111, p110, vertices, indices);
  append_triangle(p101, p001, p011, vertices, indices);
  append_triangle(p101, p011, p111, vertices, indices);
  append_triangle(p001, p000, p010, vertices, indices);
  append_triangle(p001, p010, p011, vertices, indices);
  append_triangle(p010, p110, p111, vertices, indices);
  append_triangle(p010, p111, p011, vertices, indices);
  append_triangle(p001, p101, p100, vertices, indices);
  append_triangle(p001, p100, p000, vertices, indices);
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
  emitWidgetCreatedOnce();
  QElapsedTimer timer;
  timer.start();
  incoming_visuals_ = visuals;
  last_received_visual_count_ = visuals.size();
  last_received_urdf_mesh_visual_count_ = std::count_if(
    visuals.begin(), visuals.end(),
    [](const RobotVisual &visual) {
      return is_urdf_mesh_visual(visual);
    });
  last_received_proxy_visual_count_ = std::count_if(
    visuals.begin(), visuals.end(),
    [](const RobotVisual &visual) {
      return visual.proxy_visual;
    });
  last_resolved_mesh_visual_count_ = std::count_if(
    visuals.begin(), visuals.end(),
    [](const RobotVisual &visual) {
      return is_urdf_mesh_visual(visual) && !visual.mesh_resolved_path.trimmed().isEmpty();
    });
  visuals_.clear();
  QSet<QString> active_paths;
  for (const auto &visual : visuals) {
    if (!is_accepted_opengl_mesh_visual(visual)) {
      continue;
    }
    visuals_.push_back(visual);
    active_paths.insert(visual.mesh_resolved_path.trimmed());
  }

  emitSetVisualsDiagnostics(visuals, active_paths);

  pruneInactiveMeshes(active_paths);
  for (const auto &visual : visuals_) {
    if (visual.mesh_resolved_path.isEmpty()) {
      continue;
    }
    if (mesh_cache_.find(visual.mesh_resolved_path) != mesh_cache_.end()) {
      continue;
    }

    auto mesh = std::make_unique<GpuMesh>();
    mesh->attempted = true;
    mesh->load_status = "started";
    loadStlMesh(visual, *mesh);
    emitStlLoadDiagnostics(visual, *mesh);
    if (mesh->rejected || mesh->vertices.isEmpty() || mesh->indices.isEmpty()) {
      if (!warning_cache_.contains(visual.mesh_resolved_path)) {
        warning_cache_.insert(visual.mesh_resolved_path);
        qCWarning(amrVizMeshLog).noquote() <<
          QString("OpenGL robot mesh load failed for %1: %2")
            .arg(visual.mesh_filename, mesh->error);
      }
    } else if (!success_cache_.contains(visual.mesh_resolved_path)) {
      success_cache_.insert(visual.mesh_resolved_path);
      qCInfo(amrVizMeshLog).noquote() <<
        QString("OpenGL robot mesh ready: %1 (%2 triangle(s), bbox=%3, frame=%4)")
          .arg(visual.mesh_filename)
          .arg(mesh->indices.size() / 3)
          .arg(meshBoundsText(*mesh))
          .arg(visual.frame_id);
    }
    mesh_cache_[visual.mesh_resolved_path] = std::move(mesh);
  }

  emitSetVisualsDiagnostics(visuals, active_paths);
  emitMeshVisualDiagnostics();
  emitOpenGLMeshSummary("setRobotVisuals");
  emitMeshStatusTable();

  if (timer.elapsed() > 33) {
    qCWarning(amrVizOpenGLLog).noquote() <<
      QString("OpenGL robot mesh update slow: %1 ms").arg(timer.elapsed());
  }
  setRenderVisible(hasRenderableVisuals());
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

void RobotOpenGLWidget::setRenderVisible(const bool visible)
{
  const bool changed = visible != last_render_visible_state_;
  last_render_visible_state_ = visible;
  setVisible(visible);
  if (visible) {
    raise();
    emitWidgetGeometry("setVisible(true)");
  } else if (changed) {
    qCDebug(amrVizOpenGLLog).noquote() <<
      QString("OpenGL widget visible state: visible=false, size=%1x%2, parent size=%3x%4")
        .arg(width())
        .arg(height())
        .arg(parentWidget() ? parentWidget()->width() : 0)
        .arg(parentWidget() ? parentWidget()->height() : 0);
  }
  if (!visible) {
    return;
  }
}

bool RobotOpenGLWidget::hasRenderableMesh(const RobotVisual &visual) const
{
  if (opengl_failed_) {
    return false;
  }
  if (
    !is_accepted_opengl_mesh_visual(visual))
  {
    return false;
  }
  const auto it = mesh_cache_.find(visual.mesh_resolved_path);
  return it != mesh_cache_.end() && it->second && !it->second->rejected &&
    it->second->uploaded && initialized_ && program_.isLinked();
}

bool RobotOpenGLWidget::shouldSuppressProxyForVisual(const RobotVisual &visual) const
{
  if (
    opengl_failed_ ||
    !is_accepted_opengl_mesh_visual(visual))
  {
    return false;
  }
  if (
    (visual.robot_opengl_debug_axes || visual.robot_opengl_debug_cube ||
    visual.robot_opengl_force_visible || visual.robot_opengl_stl_only_debug) && isVisible())
  {
    return true;
  }
  const auto it = mesh_cache_.find(visual.mesh_resolved_path);
  return it != mesh_cache_.end() && it->second && !it->second->rejected &&
    !it->second->vertices.isEmpty() && !it->second->indices.isEmpty();
}

bool RobotOpenGLWidget::hasRenderableVisuals() const
{
  if (opengl_failed_) {
    return false;
  }
  if (
    (debugAxesEnabled() || debugCubeEnabled() || forceVisibleEnabled() ||
    stlOnlyDebugEnabled()) && !visuals_.isEmpty())
  {
    return true;
  }
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

QString RobotOpenGLWidget::meshStatus(const RobotVisual &visual) const
{
  if (opengl_failed_) {
    return QString("OpenGL unavailable: %1").arg(opengl_failure_reason_);
  }
  if (
    !is_accepted_opengl_mesh_visual(visual))
  {
    return QString("not an accepted OpenGL mesh visual: %1").arg(opengl_candidate_skip_reason(visual));
  }

  const auto it = mesh_cache_.find(visual.mesh_resolved_path);
  if (it == mesh_cache_.end() || !it->second) {
    return "mesh load pending";
  }
  const GpuMesh &mesh = *it->second;
  if (mesh.rejected) {
    return QString("mesh rejected: %1").arg(mesh.error);
  }
  if (mesh.uploaded && initialized_ && program_.isLinked()) {
    return "rendered";
  }
  if (!initialized_) {
    return "OpenGL context pending";
  }
  if (!program_.isLinked()) {
    return "shader program unavailable";
  }
  if (mesh.vertices.isEmpty() || mesh.indices.isEmpty()) {
    return "mesh contains no drawable triangles";
  }
  return "GPU upload pending";
}

int RobotOpenGLWidget::loadedMeshCount() const
{
  int count = 0;
  for (const auto &entry : mesh_cache_) {
    if (
      entry.second &&
      !entry.second->rejected &&
      !entry.second->vertices.isEmpty() &&
      !entry.second->indices.isEmpty())
    {
      ++count;
    }
  }
  return count;
}

int RobotOpenGLWidget::rejectedMeshCount() const
{
  int count = 0;
  for (const auto &entry : mesh_cache_) {
    if (entry.second && entry.second->rejected) {
      ++count;
    }
  }
  return count;
}

bool RobotOpenGLWidget::debugAxesEnabled() const
{
  return std::any_of(
    visuals_.begin(), visuals_.end(),
    [](const RobotVisual &visual) {
      return visual.robot_opengl_debug_axes;
    });
}

bool RobotOpenGLWidget::debugCubeEnabled() const
{
  return std::any_of(
    visuals_.begin(), visuals_.end(),
    [](const RobotVisual &visual) {
      return visual.robot_opengl_debug_cube;
    });
}

bool RobotOpenGLWidget::forceVisibleEnabled() const
{
  return std::any_of(
    visuals_.begin(), visuals_.end(),
    [](const RobotVisual &visual) {
      return visual.robot_opengl_force_visible;
    });
}

bool RobotOpenGLWidget::stlOnlyDebugEnabled() const
{
  return std::any_of(
    visuals_.begin(), visuals_.end(),
    [](const RobotVisual &visual) {
      return visual.robot_opengl_stl_only_debug;
    });
}

bool RobotOpenGLWidget::debugMeshBboxEnabled() const
{
  return std::any_of(
    visuals_.begin(), visuals_.end(),
    [](const RobotVisual &visual) {
      return visual.robot_opengl_debug_mesh_bbox;
    });
}

bool RobotOpenGLWidget::debugCameraEnabled() const
{
  return std::any_of(
    visuals_.begin(), visuals_.end(),
    [](const RobotVisual &visual) {
      return visual.robot_opengl_debug_camera;
    });
}

int RobotOpenGLWidget::uploadedMeshCount() const
{
  int count = 0;
  for (const auto &entry : mesh_cache_) {
    if (entry.second && entry.second->uploaded) {
      ++count;
    }
  }
  return count;
}

void RobotOpenGLWidget::initializeGL()
{
  if (!initialize_entered_event_emitted_) {
    initialize_entered_event_emitted_ = true;
    qCDebug(amrVizOpenGLLog) << "OpenGL initializeGL entered";
  }
  initializeOpenGLFunctions();
  initialized_ = true;
  opengl_failed_ = false;
  opengl_failure_reason_.clear();
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
    "uniform vec3 material_color;\n"
    "void main() {\n"
    "  float diffuse = abs(dot(normalize(v_normal), normalize(light_dir)));\n"
    "  float shade = 0.34 + (diffuse * 0.54);\n"
    "  gl_FragColor = vec4(material_color * shade, 0.96);\n"
    "}\n";

  if (
    !program_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader) ||
    !program_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader) ||
    !program_.link())
  {
    opengl_failed_ = true;
    opengl_failure_reason_ = QString("shader setup failed: %1").arg(program_.log());
    emitFallbackOnce(opengl_failure_reason_);
    if (parentWidget()) {
      parentWidget()->update();
    }
  } else {
    const QOpenGLContext *current_context = QOpenGLContext::currentContext();
    const QSurfaceFormat context_format = current_context ? current_context->format() : format();
    qCInfo(amrVizOpenGLLog).noquote() <<
      QString("OpenGL available: version %1.%2, depth testing enabled, fallback backend=proxy")
        .arg(context_format.majorVersion())
        .arg(context_format.minorVersion());
    Q_EMIT visualizationEvent(
      "OpenGL robot renderer enabled; detailed diagnostics are available in terminal logs");
  }
}

void RobotOpenGLWidget::resizeGL(int, int)
{
  emitWidgetGeometry("resizeGL");
  update();
}

void RobotOpenGLWidget::paintGL()
{
  if (!paint_entered_event_emitted_) {
    paint_entered_event_emitted_ = true;
    qCDebug(amrVizOpenGLLog) << "OpenGL paintGL entered";
  }
  glViewport(0, 0, width(), height());
  glClearColor(0.0F, 0.0F, 0.0F, 0.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (opengl_failed_) {
    emitPaintDiagnostics(0, 0, 0, 0, 0, 0, 0, glGetError());
    return;
  }
  if (!program_.isLinked()) {
    emitPaintDiagnostics(0, 0, 0, 0, 0, 0, 0, glGetError());
    return;
  }

  const QVector3D light_dir =
    (cameraUp() - cameraForward() + QVector3D(0.25F, 0.20F, 0.35F)).normalized();

  program_.bind();
  program_.setUniformValue("light_dir", light_dir);
  int draw_calls = 0;
  int debug_axis_draw_count = 0;
  int debug_cube_draw_count = 0;
  int rendered_triangles = 0;
  int rendered_stl_triangles = 0;
  int stl_draw_calls = 0;
  int fallback_cube_draw_calls = 0;
  GLenum accumulated_draw_error = GL_NO_ERROR;
  for (const auto &visual : visuals_) {
    const int previous_axis_draw_count = debug_axis_draw_count;
    const int previous_cube_draw_count = debug_cube_draw_count;
    drawDebugGeometryForVisual(
      visual,
      debug_axis_draw_count,
      debug_cube_draw_count,
      rendered_triangles);
    draw_calls +=
      (debug_axis_draw_count - previous_axis_draw_count) +
      (debug_cube_draw_count - previous_cube_draw_count);

    auto it = mesh_cache_.find(visual.mesh_resolved_path);
    if (it == mesh_cache_.end() || !it->second || it->second->rejected) {
      continue;
    }
    GpuMesh &mesh = *it->second;
    if (visual.robot_opengl_debug_mesh_bbox && mesh.validation_status == "passed") {
      drawMeshBoundingBox(visual, mesh, draw_calls, rendered_triangles);
    }
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
    drawMesh(mesh, model, QVector3D(0.72F, 0.74F, 0.77F));
    if (mesh.draw_error_code != GL_NO_ERROR) {
      accumulated_draw_error = mesh.draw_error_code;
    }
    ++draw_calls;
    ++stl_draw_calls;
    const int stl_triangle_count = mesh.indices.size() / 3;
    rendered_triangles += stl_triangle_count;
    rendered_stl_triangles += stl_triangle_count;
    if (uploaded_now && parentWidget()) {
      parentWidget()->update();
    }

    const QString render_key = visual.mesh_resolved_path;
    if (!render_event_cache_.contains(render_key)) {
      render_event_cache_.insert(render_key);
      qCInfo(amrVizMeshLog).noquote() <<
        QString("OpenGL robot mesh rendered: %1 (%2 triangle(s))")
          .arg(visual.mesh_filename)
          .arg(mesh.indices.size() / 3);
    }
  }
  if (stlOnlyDebugEnabled() && stl_draw_calls == 0) {
    drawStlOnlyFallbackCube(draw_calls, fallback_cube_draw_calls, rendered_triangles);
  }
  const GLenum final_error = glGetError();
  const GLenum draw_error = accumulated_draw_error != GL_NO_ERROR ? accumulated_draw_error : final_error;
  last_draw_calls_ = draw_calls;
  last_stl_draw_calls_ = stl_draw_calls;
  last_fallback_cube_draw_calls_ = fallback_cube_draw_calls;
  last_rendered_triangles_ = rendered_triangles;
  last_rendered_stl_triangles_ = rendered_stl_triangles;
  last_draw_error_code_ = draw_error;
  program_.release();
  emitPaintDiagnostics(
    draw_calls,
    stl_draw_calls,
    fallback_cube_draw_calls,
    debug_axis_draw_count,
    debug_cube_draw_count,
    rendered_stl_triangles,
    rendered_triangles,
    draw_error);
  emitOpenGLMeshSummary("paintGL");
  emitMeshStatusTable();
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

QVector3D RobotOpenGLWidget::effectiveFocalPoint() const
{
  if (!debugCameraEnabled() || visuals_.isEmpty()) {
    return focal_point_;
  }

  QVector3D sum(0.0F, 0.0F, 0.0F);
  int count = 0;
  for (const auto &visual : visuals_) {
    const auto it = mesh_cache_.find(visual.mesh_resolved_path);
    if (it != mesh_cache_.end() && it->second && !it->second->rejected) {
      const QVector3D local_center = (it->second->min_bounds + it->second->max_bounds) * 0.5F;
      sum += poseMatrixForVisual(visual).map(local_center);
    } else {
      sum += QVector3D(
        static_cast<float>(visual.pose.x),
        static_cast<float>(visual.pose.y),
        static_cast<float>(visual.pose.z));
    }
    ++count;
  }
  return count > 0 ? sum / static_cast<float>(count) : focal_point_;
}

double RobotOpenGLWidget::effectiveCameraDistance() const
{
  if (!debugCameraEnabled()) {
    return camera_distance_;
  }
  double max_diagonal = 0.5;
  for (const auto &entry : mesh_cache_) {
    if (entry.second && !entry.second->rejected) {
      max_diagonal = std::max(max_diagonal, entry.second->diagonal);
    }
  }
  return std::clamp(max_diagonal * 4.0, 1.0, 10.0);
}

double RobotOpenGLWidget::effectivePixelsPerMeter() const
{
  if (!debugCameraEnabled()) {
    return pixels_per_meter_;
  }
  double max_extent = 0.5;
  for (const auto &entry : mesh_cache_) {
    if (entry.second && !entry.second->rejected) {
      max_extent = std::max(max_extent, static_cast<double>(entry.second->extent.length()));
    }
  }
  const double viewport_size = std::max(1, std::min(width(), height()));
  return std::clamp(viewport_size / std::max(max_extent * 3.0, 0.5), 80.0, 600.0);
}

QMatrix4x4 RobotOpenGLWidget::poseMatrixForVisual(const RobotVisual &visual) const
{
  QMatrix4x4 model;
  model.translate(
    static_cast<float>(visual.pose.x),
    static_cast<float>(visual.pose.y),
    static_cast<float>(visual.pose.z));
  model.rotate(static_cast<float>(visual.pose.yaw * 180.0 / k_pi), 0.0F, 0.0F, 1.0F);
  model.rotate(static_cast<float>(visual.pose.pitch * 180.0 / k_pi), 0.0F, 1.0F, 0.0F);
  model.rotate(static_cast<float>(visual.pose.roll * 180.0 / k_pi), 1.0F, 0.0F, 0.0F);
  return model;
}

QMatrix4x4 RobotOpenGLWidget::modelMatrixForVisual(const RobotVisual &visual) const
{
  QMatrix4x4 model = poseMatrixForVisual(visual);
  // RobotVisual::pose is already world/link transform composed with the URDF visual origin.
  // STL vertices stay in raw file units; this matrix applies URDF mesh scale exactly once.
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
  const QVector3D focal_point = effectiveFocalPoint();
  const QVector3D eye =
    focal_point - (cameraForward() * static_cast<float>(std::max(effectiveCameraDistance(), 0.1)));
  view.lookAt(eye, focal_point, cameraUp());
  return view;
}

QMatrix4x4 RobotOpenGLWidget::projectionMatrix() const
{
  QMatrix4x4 projection;
  const double pixels_per_meter = std::max(effectivePixelsPerMeter(), 1.0);
  const float half_width = static_cast<float>(width() / (2.0 * pixels_per_meter));
  const float half_height = static_cast<float>(height() / (2.0 * pixels_per_meter));
  const float far_plane = static_cast<float>(std::max(effectiveCameraDistance() + 100.0, 120.0));
  projection.ortho(-half_width, half_width, -half_height, half_height, 0.01F, far_plane);
  return projection;
}

bool RobotOpenGLWidget::loadStlMesh(const RobotVisual &visual, GpuMesh &mesh)
{
  const QString path = visual.mesh_resolved_path.trimmed();
  mesh.source_path = path;
  mesh.load_status = "started";
  if (path.isEmpty()) {
    mesh.rejected = true;
    mesh.error = "empty resolved mesh path";
    mesh.load_status = "rejected";
    mesh.validation_status = "not_run";
    return false;
  }
  const QFileInfo file_info(path);
  if (file_info.suffix().compare("stl", Qt::CaseInsensitive) != 0) {
    mesh.rejected = true;
    mesh.error = QString("unsupported extension .%1").arg(file_info.suffix());
    mesh.load_status = "rejected";
    mesh.validation_status = "not_run";
    return false;
  }

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    mesh.rejected = true;
    mesh.error = file.errorString();
    mesh.load_status = "rejected";
    mesh.validation_status = "not_run";
    return false;
  }

  const qint64 file_size = file.size();
  mesh.file_size_bytes = file_size;
  const qint64 max_file_size_bytes =
    static_cast<qint64>(std::max(visual.mesh_max_file_size_mb, 1)) * 1024LL * 1024LL;
  if (file_size <= 0) {
    mesh.rejected = true;
    mesh.error = QString("file size rejection: empty STL file (%1 bytes)").arg(file_size);
    mesh.load_status = "rejected";
    mesh.validation_status = "not_run";
    return false;
  }
  if (file_size > max_file_size_bytes) {
    mesh.rejected = true;
    mesh.error = QString("file size rejection: STL file size %1 bytes exceeds %2 bytes (%3 MiB) limit")
      .arg(file_size)
      .arg(max_file_size_bytes)
      .arg(visual.mesh_max_file_size_mb);
    mesh.load_status = "rejected";
    mesh.validation_status = "not_run";
    return false;
  }

  if (file_size >= 84) {
    file.seek(80);
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    quint32 triangle_count = 0;
    stream >> triangle_count;
    const qint64 expected_size = 84 + (static_cast<qint64>(triangle_count) * 50);
    if (triangle_count > 0 && expected_size == file_size) {
      mesh.stl_format = "binary";
      mesh.source_triangle_count = triangle_count;
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
      mesh.loaded_triangle_count = static_cast<quint32>(mesh.indices.size() / 3);
      if (mesh.loaded_triangle_count != loaded_count) {
        mesh.rejected = true;
        mesh.error = QString("invalid binary STL parsing: loaded %1 of expected %2 triangle(s)")
          .arg(mesh.loaded_triangle_count)
          .arg(loaded_count);
        mesh.load_status = "rejected";
        mesh.validation_status = "not_run";
        return false;
      }
      if (triangle_count > loaded_count) {
        qCInfo(amrVizMeshLog).noquote() <<
          QString("OpenGL robot mesh triangle load capped: loaded %1 of %2")
            .arg(loaded_count)
            .arg(triangle_count);
      }
      mesh.load_status = mesh.indices.isEmpty() ? "rejected" : "loaded";
      return validateMesh(visual, mesh);
    }
  }

  file.seek(0);
  const QByteArray header = file.peek(512).trimmed();
  if (!header.startsWith("solid")) {
    mesh.rejected = true;
    mesh.error = "invalid STL parsing: not a recognized binary or ASCII STL";
    mesh.load_status = "rejected";
    mesh.validation_status = "not_run";
    return false;
  }

  const QString content = QString::fromLatin1(file.readAll());
  mesh.stl_format = "ascii";
  QVector<QVector3D> triangle_vertices;
  triangle_vertices.reserve(3);
  quint32 parsed_triangles = 0;
  const quint32 max_loaded_triangles =
    static_cast<quint32>(std::max(visual.mesh_max_loaded_triangles, 1));
  const QStringList lines = content.split('\n');
  for (const QString &line : lines) {
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
      if (parsed_triangles < max_loaded_triangles) {
        append_triangle(
          triangle_vertices[0],
          triangle_vertices[1],
          triangle_vertices[2],
          mesh.vertices,
          mesh.indices);
      }
      triangle_vertices.clear();
      ++parsed_triangles;
    }
  }
  mesh.source_triangle_count = parsed_triangles;
  mesh.loaded_triangle_count = static_cast<quint32>(mesh.indices.size() / 3);

  if (mesh.indices.isEmpty()) {
    mesh.rejected = true;
    mesh.error = "zero triangle count: not a supported ASCII or binary STL";
    mesh.load_status = "rejected";
    mesh.validation_status = "not_run";
    return false;
  }
  if (parsed_triangles >= max_loaded_triangles) {
    qCInfo(amrVizMeshLog).noquote() <<
      QString("OpenGL robot mesh triangle load capped: loaded first %1 ASCII triangle(s)")
        .arg(max_loaded_triangles);
  }
  mesh.load_status = "loaded";
  return validateMesh(visual, mesh);
}

bool RobotOpenGLWidget::validateMesh(const RobotVisual &visual, GpuMesh &mesh)
{
  mesh.validation_status = "started";
  mesh.validation_max_abs_coordinate = std::max(visual.mesh_max_abs_coordinate_m, 0.001);
  mesh.validation_max_extent = std::max(visual.mesh_max_extent_m, 0.001);
  mesh.validation_scale_applied = true;
  mesh.model_matrix_applies_scale = true;
  if (mesh.vertices.isEmpty() || mesh.indices.isEmpty()) {
    mesh.rejected = true;
    mesh.error = "triangle count is zero";
    mesh.validation_status = "rejected";
    return false;
  }

  float raw_min_x = std::numeric_limits<float>::infinity();
  float raw_min_y = std::numeric_limits<float>::infinity();
  float raw_min_z = std::numeric_limits<float>::infinity();
  float raw_max_x = -std::numeric_limits<float>::infinity();
  float raw_max_y = -std::numeric_limits<float>::infinity();
  float raw_max_z = -std::numeric_limits<float>::infinity();
  float scaled_min_x = std::numeric_limits<float>::infinity();
  float scaled_min_y = std::numeric_limits<float>::infinity();
  float scaled_min_z = std::numeric_limits<float>::infinity();
  float scaled_max_x = -std::numeric_limits<float>::infinity();
  float scaled_max_y = -std::numeric_limits<float>::infinity();
  float scaled_max_z = -std::numeric_limits<float>::infinity();
  bool finite = true;
  bool coordinate_in_range = true;
  const float max_abs_coordinate = static_cast<float>(mesh.validation_max_abs_coordinate);

  for (int i = 0; i + 2 < mesh.vertices.size(); i += 6) {
    const QVector3D raw_point(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]);
    const QVector3D point = scaled_mesh_point(visual, raw_point);
    if (!finite_vector(raw_point) || !finite_vector(point)) {
      finite = false;
      break;
    }
    raw_min_x = std::min(raw_min_x, raw_point.x());
    raw_min_y = std::min(raw_min_y, raw_point.y());
    raw_min_z = std::min(raw_min_z, raw_point.z());
    raw_max_x = std::max(raw_max_x, raw_point.x());
    raw_max_y = std::max(raw_max_y, raw_point.y());
    raw_max_z = std::max(raw_max_z, raw_point.z());
    if (
      std::abs(point.x()) > max_abs_coordinate ||
      std::abs(point.y()) > max_abs_coordinate ||
      std::abs(point.z()) > max_abs_coordinate)
    {
      coordinate_in_range = false;
    }
    scaled_min_x = std::min(scaled_min_x, point.x());
    scaled_min_y = std::min(scaled_min_y, point.y());
    scaled_min_z = std::min(scaled_min_z, point.z());
    scaled_max_x = std::max(scaled_max_x, point.x());
    scaled_max_y = std::max(scaled_max_y, point.y());
    scaled_max_z = std::max(scaled_max_z, point.z());
  }

  mesh.validation_finite_coordinates = finite;
  mesh.validation_coordinate_in_range = coordinate_in_range;
  mesh.raw_min_bounds = QVector3D(raw_min_x, raw_min_y, raw_min_z);
  mesh.raw_max_bounds = QVector3D(raw_max_x, raw_max_y, raw_max_z);
  mesh.raw_extent = mesh.raw_max_bounds - mesh.raw_min_bounds;
  mesh.raw_diagonal = static_cast<double>(mesh.raw_extent.length());
  mesh.min_bounds = QVector3D(scaled_min_x, scaled_min_y, scaled_min_z);
  mesh.max_bounds = QVector3D(scaled_max_x, scaled_max_y, scaled_max_z);
  mesh.extent = mesh.max_bounds - mesh.min_bounds;
  mesh.diagonal = static_cast<double>(mesh.extent.length());
  if (!finite) {
    mesh.rejected = true;
    mesh.error = "non-finite coordinate";
  } else if (!coordinate_in_range) {
    mesh.rejected = true;
    mesh.error =
      QString("scaled coordinate exceeds %1 m absolute limit").arg(mesh.validation_max_abs_coordinate);
  } else if (!std::isfinite(mesh.diagonal) || mesh.diagonal <= 0.0) {
    mesh.rejected = true;
    mesh.error = "invalid scaled bbox diagonal";
  } else if (mesh.diagonal > mesh.validation_max_extent) {
    mesh.rejected = true;
    mesh.error =
      QString("scaled bbox diagonal %1 m exceeds %2 m").arg(mesh.diagonal).arg(mesh.validation_max_extent);
  }

  if (mesh.rejected) {
    mesh.validation_status = "rejected";
    qCWarning(amrVizMeshLog).noquote() <<
      QString("OpenGL STL validation rejected: uri=%1, path=%2, reason=%3, raw_bbox_min=%4, raw_bbox_max=%5, scaled_bbox_min=%6, scaled_bbox_max=%7, raw_bbox_diagonal=%8, scaled_bbox_diagonal=%9, max_abs_coordinate_threshold=%10, max_extent_threshold=%11, finite_coordinates=%12, coordinate_in_range=%13, source_triangle_count=%14, loaded_triangle_count=%15, loaded_vertex_count=%16, file_size=%17, mesh_scale=%18 %19 %20, unit_scale=%21, scale_applied_during_validation=%22, scale_will_be_applied_in_modelMatrix=%23")
        .arg(visual.mesh_filename)
        .arg(mesh.source_path)
        .arg(mesh.error)
        .arg(vector_text(mesh.raw_min_bounds))
        .arg(vector_text(mesh.raw_max_bounds))
        .arg(vector_text(mesh.min_bounds))
        .arg(vector_text(mesh.max_bounds))
        .arg(mesh.raw_diagonal)
        .arg(mesh.diagonal)
        .arg(mesh.validation_max_abs_coordinate)
        .arg(mesh.validation_max_extent)
        .arg(mesh.validation_finite_coordinates ? "true" : "false")
        .arg(mesh.validation_coordinate_in_range ? "true" : "false")
        .arg(mesh.source_triangle_count)
        .arg(mesh.loaded_triangle_count)
        .arg(mesh.vertices.size() / 6)
        .arg(mesh.file_size_bytes)
        .arg(visual.mesh_scale_x)
        .arg(visual.mesh_scale_y)
        .arg(visual.mesh_scale_z)
        .arg(visual.mesh_auto_unit_scale ? visual.mesh_unit_scale : 1.0)
        .arg(mesh.validation_scale_applied ? "true" : "false")
        .arg(mesh.model_matrix_applies_scale ? "true" : "false");
    mesh.vertices.clear();
    mesh.indices.clear();
    return false;
  }
  mesh.validation_status = "passed";
  return true;
}

bool RobotOpenGLWidget::uploadMesh(GpuMesh &mesh)
{
  mesh.upload_status = "started";
  if (opengl_failed_ || !initialized_ || mesh.vertices.isEmpty() || mesh.indices.isEmpty()) {
    mesh.upload_status = "not_ready";
    return false;
  }
  if (!program_.isLinked()) {
    mesh.rejected = true;
    mesh.error = "shader program is not linked";
    mesh.upload_status = "failed";
    emitFallbackOnce(mesh.error);
    return false;
  }
  if (!mesh.vertex_buffer.create() || !mesh.index_buffer.create() || !mesh.vao.create()) {
    mesh.rejected = true;
    mesh.error = "failed to create OpenGL buffers";
    mesh.upload_status = "failed";
    destroyMeshBuffers(mesh);
    emitFallbackOnce(mesh.error);
    return false;
  }

  program_.bind();
  const int position_location = program_.attributeLocation("position");
  const int normal_location = program_.attributeLocation("normal");
  if (position_location < 0 || normal_location < 0) {
    mesh.rejected = true;
    mesh.error = "shader attribute location missing";
    mesh.upload_status = "failed";
    destroyMeshBuffers(mesh);
    program_.release();
    emitFallbackOnce(mesh.error);
    return false;
  }
  {
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
  }
  mesh.index_buffer.release();
  const GLenum upload_error = glGetError();
  mesh.upload_error_code = upload_error;
  program_.release();
  mesh.uploaded = true;
  mesh.upload_status = upload_error == GL_NO_ERROR ? "uploaded" : "gl_error";
  emitUploadDiagnostics(mesh, upload_error);
  return true;
}

bool RobotOpenGLWidget::uploadDebugCube()
{
  if (debug_cube_mesh_.uploaded) {
    return true;
  }
  if (debug_cube_mesh_.vertices.isEmpty()) {
    debug_cube_mesh_.source_path = "<debug_cube>";
    append_box(debug_cube_mesh_.vertices, debug_cube_mesh_.indices);
    debug_cube_mesh_.loaded_triangle_count = static_cast<quint32>(debug_cube_mesh_.indices.size() / 3);
    debug_cube_mesh_.source_triangle_count = debug_cube_mesh_.loaded_triangle_count;
    debug_cube_mesh_.min_bounds = QVector3D(-0.5F, -0.5F, -0.5F);
    debug_cube_mesh_.max_bounds = QVector3D(0.5F, 0.5F, 0.5F);
    debug_cube_mesh_.extent = QVector3D(1.0F, 1.0F, 1.0F);
    debug_cube_mesh_.diagonal = static_cast<double>(debug_cube_mesh_.extent.length());
  }
  return uploadMesh(debug_cube_mesh_);
}

void RobotOpenGLWidget::drawMesh(
  GpuMesh &mesh,
  const QMatrix4x4 &model,
  const QVector3D &color)
{
  if (!program_.isLinked()) {
    return;
  }
  program_.bind();
  program_.setUniformValue("mvp", projectionMatrix() * viewMatrix() * model);
  program_.setUniformValue("normal_matrix", model.normalMatrix());
  program_.setUniformValue("material_color", color);
  QOpenGLVertexArrayObject::Binder vao_binder(&mesh.vao);
  glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.indices.size()), GL_UNSIGNED_INT, nullptr);
  mesh.draw_error_code = glGetError();
  mesh.draw_status = mesh.draw_error_code == GL_NO_ERROR ? "drawn" : "gl_error";
  ++mesh.draw_call_count;
}

void RobotOpenGLWidget::drawMeshBoundingBox(
  const RobotVisual &visual,
  const GpuMesh &mesh,
  int &draw_calls,
  int &rendered_triangles)
{
  if (!uploadDebugCube() || !debug_cube_mesh_.vao.isCreated()) {
    return;
  }

  const QVector3D min_bound = mesh.min_bounds;
  const QVector3D max_bound = mesh.max_bounds;
  const QVector3D extent = max_bound - min_bound;
  if (!finite_vector(min_bound) || !finite_vector(max_bound) || extent.lengthSquared() <= 0.0F) {
    return;
  }

  const float thickness = std::max(static_cast<float>(mesh.diagonal) * 0.01F, 0.006F);
  const QMatrix4x4 pose = poseMatrixForVisual(visual);
  const QVector3D color(1.0F, 0.18F, 0.12F);
  auto draw_edge = [&](const QVector3D &center, const QVector3D &scale) {
      QMatrix4x4 edge = pose;
      edge.translate(center);
      edge.scale(
        std::max(scale.x(), thickness),
        std::max(scale.y(), thickness),
        std::max(scale.z(), thickness));
      drawMesh(debug_cube_mesh_, edge, color);
      ++draw_calls;
      rendered_triangles += debug_cube_mesh_.indices.size() / 3;
    };

  for (const float y : {min_bound.y(), max_bound.y()}) {
    for (const float z : {min_bound.z(), max_bound.z()}) {
      draw_edge(
        QVector3D((min_bound.x() + max_bound.x()) * 0.5F, y, z),
        QVector3D(std::max(extent.x(), thickness), thickness, thickness));
    }
  }
  for (const float x : {min_bound.x(), max_bound.x()}) {
    for (const float z : {min_bound.z(), max_bound.z()}) {
      draw_edge(
        QVector3D(x, (min_bound.y() + max_bound.y()) * 0.5F, z),
        QVector3D(thickness, std::max(extent.y(), thickness), thickness));
    }
  }
  for (const float x : {min_bound.x(), max_bound.x()}) {
    for (const float y : {min_bound.y(), max_bound.y()}) {
      draw_edge(
        QVector3D(x, y, (min_bound.z() + max_bound.z()) * 0.5F),
        QVector3D(thickness, thickness, std::max(extent.z(), thickness)));
    }
  }
}

void RobotOpenGLWidget::drawStlOnlyFallbackCube(
  int &draw_calls,
  int &fallback_cube_draw_calls,
  int &rendered_triangles)
{
  if (!uploadDebugCube() || !debug_cube_mesh_.vao.isCreated() || visuals_.isEmpty()) {
    return;
  }

  const RobotVisual &visual = visuals_.front();
  const float size = static_cast<float>(std::clamp(visual.robot_opengl_debug_size_m, 0.01, 5.0));
  QMatrix4x4 cube = poseMatrixForVisual(visual);
  cube.scale(size, size, size);
  drawMesh(debug_cube_mesh_, cube, QVector3D(1.0F, 0.04F, 0.04F));
  ++draw_calls;
  ++fallback_cube_draw_calls;
  rendered_triangles += debug_cube_mesh_.indices.size() / 3;
  const QString reason = QString("accepted_opengl_meshes=%1, loaded_meshes=%2, validation_rejected_meshes=%3, uploaded_meshes=%4, stl_draw_calls=0, fallback_cube_draw_calls=%5")
    .arg(visuals_.size())
    .arg(loadedMeshCount())
    .arg(rejectedMeshCount())
    .arg(uploadedMeshCount())
    .arg(fallback_cube_draw_calls);
  if (reason != last_stl_fallback_reason_) {
    const bool first_fallback = last_stl_fallback_reason_.isEmpty();
    last_stl_fallback_reason_ = reason;
    qCWarning(amrVizOpenGLLog).noquote() <<
      QString("OpenGL STL-only fallback: no STL mesh was drawn; rendered red fallback cube (%1)")
        .arg(reason);
    if (first_fallback) {
      Q_EMIT visualizationEvent("OpenGL STL mesh draw unavailable; showing fallback cube");
    }
  }
}

void RobotOpenGLWidget::drawDebugGeometryForVisual(
  const RobotVisual &visual,
  int &axis_draw_count,
  int &cube_draw_count,
  int &rendered_triangles)
{
  if (!visual.robot_opengl_debug_axes && !visual.robot_opengl_debug_cube) {
    return;
  }
  if (!uploadDebugCube() || !debug_cube_mesh_.vao.isCreated()) {
    return;
  }

  const float size = static_cast<float>(std::clamp(visual.robot_opengl_debug_size_m, 0.01, 5.0));
  const float half_axis = size * 0.5F;
  const float axis_length = size;
  const float axis_thickness = std::max(size * 0.08F, 0.01F);
  QMatrix4x4 base = poseMatrixForVisual(visual);

  if (visual.robot_opengl_debug_cube) {
    QMatrix4x4 cube = base;
    cube.scale(size, size, size);
    drawMesh(debug_cube_mesh_, cube, QVector3D(1.0F, 0.86F, 0.12F));
    ++cube_draw_count;
    rendered_triangles += debug_cube_mesh_.indices.size() / 3;
  }

  if (!visual.robot_opengl_debug_axes) {
    return;
  }

  QMatrix4x4 x_axis = base;
  x_axis.translate(half_axis, 0.0F, 0.0F);
  x_axis.scale(axis_length, axis_thickness, axis_thickness);
  drawMesh(debug_cube_mesh_, x_axis, QVector3D(1.0F, 0.05F, 0.04F));
  ++axis_draw_count;
  rendered_triangles += debug_cube_mesh_.indices.size() / 3;

  QMatrix4x4 y_axis = base;
  y_axis.translate(0.0F, half_axis, 0.0F);
  y_axis.scale(axis_thickness, axis_length, axis_thickness);
  drawMesh(debug_cube_mesh_, y_axis, QVector3D(0.05F, 1.0F, 0.16F));
  ++axis_draw_count;
  rendered_triangles += debug_cube_mesh_.indices.size() / 3;

  QMatrix4x4 z_axis = base;
  z_axis.translate(0.0F, 0.0F, half_axis);
  z_axis.scale(axis_thickness, axis_thickness, axis_length);
  drawMesh(debug_cube_mesh_, z_axis, QVector3D(0.08F, 0.32F, 1.0F));
  ++axis_draw_count;
  rendered_triangles += debug_cube_mesh_.indices.size() / 3;
}

void RobotOpenGLWidget::emitMeshVisualDiagnostics()
{
  const int loaded_meshes = loadedMeshCount();
  const int rejected_meshes = rejectedMeshCount();
  const QString summary = QString("%1:%2:%3:%4")
    .arg(visuals_.size())
    .arg(static_cast<int>(mesh_cache_.size()))
    .arg(loaded_meshes)
    .arg(rejected_meshes);
  if (summary != last_mesh_summary_) {
    last_mesh_summary_ = summary;
    qCInfo(amrVizMeshLog).noquote() <<
      QString("Robot description mesh render summary: backend=opengl, visual elements=%1, loaded meshes=%2, rejected meshes=%3")
        .arg(visuals_.size())
        .arg(loaded_meshes)
        .arg(rejected_meshes);
    if (!visuals_.isEmpty() && loaded_meshes == 0) {
      Q_EMIT visualizationEvent("No OpenGL robot meshes loaded; using proxy renderer");
    }
  }

  for (const auto &visual : visuals_) {
    const QString key =
      QString("%1|%2|%3").arg(visual.frame_id, visual.mesh_filename, visual.mesh_resolved_path);
    if (visual_event_cache_.contains(key)) {
      continue;
    }
    const auto it = mesh_cache_.find(visual.mesh_resolved_path);
    if (it == mesh_cache_.end() || !it->second) {
      continue;
    }
    const GpuMesh &mesh = *it->second;
    visual_event_cache_.insert(key);
    qCInfo(amrVizMeshLog).noquote() <<
      QString("Mesh visual load result: frame=%1, uri=%2, resolved=%3, scale=%4 %5 %6, triangle count=%7, backend=opengl, status=%8%9")
        .arg(visual.frame_id)
        .arg(visual.mesh_filename)
        .arg(visual.mesh_resolved_path.isEmpty() ? QString("<unresolved>") : visual.mesh_resolved_path)
        .arg(visual.mesh_scale_x)
        .arg(visual.mesh_scale_y)
        .arg(visual.mesh_scale_z)
        .arg(mesh.indices.size() / 3)
        .arg(mesh.rejected ? QString("rejected") : QString("loaded"))
        .arg(mesh.error.isEmpty() ? QString() : QString(", reason=%1").arg(mesh.error));
  }
}

void RobotOpenGLWidget::emitSetVisualsDiagnostics(
  const QVector<RobotVisual> &incoming,
  const QSet<QString> &active_paths)
{
  QStringList path_list;
  path_list.reserve(active_paths.size());
  for (const auto &path : active_paths) {
    path_list.push_back(path);
  }
  std::sort(path_list.begin(), path_list.end());
  const QString summary = QString("%1:%2:%3:%4:%5")
    .arg(incoming.size())
    .arg(visuals_.size())
    .arg(active_paths.size())
    .arg(static_cast<int>(mesh_cache_.size()))
    .arg(path_list.join("|"));
  if (summary == last_set_visuals_summary_) {
    return;
  }
  last_set_visuals_summary_ = summary;
  qCDebug(amrVizOpenGLLog).noquote() <<
    QString("OpenGL setRobotVisuals diagnostics: total_visuals=%1, accepted_opengl_meshes=%2, active mesh paths=[%3], mesh cache size=%4, loadedMeshCount=%5, rejectedMeshCount=%6")
      .arg(incoming.size())
      .arg(visuals_.size())
      .arg(path_list.join(", "))
      .arg(static_cast<int>(mesh_cache_.size()))
      .arg(loadedMeshCount())
      .arg(rejectedMeshCount());
}

void RobotOpenGLWidget::emitWidgetCreatedOnce()
{
  if (widget_created_event_emitted_) {
    return;
  }
  widget_created_event_emitted_ = true;
  qCDebug(amrVizOpenGLLog).noquote() <<
    QString("OpenGL widget created: size=%1x%2, parent size=%3x%4, visible=%5")
      .arg(width())
      .arg(height())
      .arg(parentWidget() ? parentWidget()->width() : 0)
      .arg(parentWidget() ? parentWidget()->height() : 0)
      .arg(isVisible() ? "true" : "false");
}

void RobotOpenGLWidget::emitWidgetGeometry(const QString &reason)
{
  const QString geometry_text = QString("%1,%2 %3x%4 parent=%5x%6 visible=%7")
    .arg(geometry().x())
    .arg(geometry().y())
    .arg(geometry().width())
    .arg(geometry().height())
    .arg(parentWidget() ? parentWidget()->width() : 0)
    .arg(parentWidget() ? parentWidget()->height() : 0)
    .arg(isVisible() ? "true" : "false");
  if (geometry_text == last_visible_geometry_) {
    return;
  }
  last_visible_geometry_ = geometry_text;
  qCDebug(amrVizOpenGLLog).noquote() <<
    QString("OpenGL widget geometry (%1): %2").arg(reason, geometry_text);
}

void RobotOpenGLWidget::emitStlLoadDiagnostics(
  const RobotVisual &visual,
  const GpuMesh &mesh)
{
  const QString key = QString("stl_load:%1:%2:%3:%4")
    .arg(visual.frame_id)
    .arg(visual.mesh_filename)
    .arg(visual.mesh_resolved_path)
    .arg(mesh.error);
  if (stl_load_event_cache_.contains(key)) {
    return;
  }
  stl_load_event_cache_.insert(key);
  qCInfo(amrVizMeshLog).noquote() <<
    QString("OpenGL STL load diagnostics: path=%1, file_access={%2}, file_size=%3, detected=%4, source_triangle_count=%5, loaded_triangle_count=%6, loaded_vertex_count=%7, raw_bbox_min=%8, raw_bbox_max=%9, scaled_bbox_min=%10, scaled_bbox_max=%11, raw_bbox_diagonal=%12, scaled_bbox_diagonal=%13, scaled_extent=%14, max_loaded_triangles=%15, max_file_size_mb=%16, max_extent_threshold=%17, max_abs_coordinate_threshold=%18, finite_coordinates=%19, coordinate_in_range=%20, mesh_scale=%21 %22 %23, unit_scale=%24, scale_applied_during_validation=%25, scale_will_be_applied_in_modelMatrix=%26, status=%27%28")
      .arg(visual.mesh_resolved_path)
      .arg(file_access_diagnostics_text(visual.mesh_filename, visual.mesh_resolved_path))
      .arg(mesh.file_size_bytes)
      .arg(mesh.stl_format)
      .arg(mesh.source_triangle_count)
      .arg(mesh.loaded_triangle_count)
      .arg(mesh.vertices.size() / 6)
      .arg(vector_text(mesh.raw_min_bounds))
      .arg(vector_text(mesh.raw_max_bounds))
      .arg(vector_text(mesh.min_bounds))
      .arg(vector_text(mesh.max_bounds))
      .arg(mesh.raw_diagonal)
      .arg(mesh.diagonal)
      .arg(vector_text(mesh.extent))
      .arg(visual.mesh_max_loaded_triangles)
      .arg(visual.mesh_max_file_size_mb)
      .arg(mesh.validation_max_extent > 0.0 ? mesh.validation_max_extent : visual.mesh_max_extent_m)
      .arg(mesh.validation_max_abs_coordinate > 0.0 ? mesh.validation_max_abs_coordinate : visual.mesh_max_abs_coordinate_m)
      .arg(mesh.validation_finite_coordinates ? "true" : "false")
      .arg(mesh.validation_coordinate_in_range ? "true" : "false")
      .arg(visual.mesh_scale_x)
      .arg(visual.mesh_scale_y)
      .arg(visual.mesh_scale_z)
      .arg(visual.mesh_auto_unit_scale ? visual.mesh_unit_scale : 1.0)
      .arg(mesh.validation_scale_applied ? "true" : "false")
      .arg(mesh.model_matrix_applies_scale ? "true" : "false")
      .arg(mesh.rejected ? "rejected" : "loaded")
      .arg(mesh.error.isEmpty() ? QString() : QString(", reason=%1").arg(mesh.error));
}

void RobotOpenGLWidget::emitUploadDiagnostics(const GpuMesh &mesh, const GLenum error_code)
{
  const QString key = QString("upload:%1:%2:%3")
    .arg(mesh.source_path)
    .arg(mesh.indices.size())
    .arg(error_code);
  if (upload_event_cache_.contains(key)) {
    return;
  }
  upload_event_cache_.insert(key);
  qCInfo(amrVizOpenGLLog).noquote() <<
    QString("OpenGL mesh upload diagnostics: path=%1, vao=%2, vbo=%3, ibo=%4, index count=%5, shader linked=%6, gl_error=%7")
      .arg(mesh.source_path)
      .arg(mesh.vao.isCreated() ? "true" : "false")
      .arg(mesh.vertex_buffer.isCreated() ? "true" : "false")
      .arg(mesh.index_buffer.isCreated() ? "true" : "false")
      .arg(mesh.indices.size())
      .arg(program_.isLinked() ? "true" : "false")
      .arg(static_cast<unsigned int>(error_code));
}

void RobotOpenGLWidget::emitMeshStatusTable()
{
  QStringList rows;
  rows.reserve(incoming_visuals_.size());
  for (const auto &visual : incoming_visuals_) {
    if (visual.proxy_visual) {
      rows.push_back(
        QString("frame_id=%1 | uri=<proxy> | resolved_path=<empty> | accepted_for_opengl=false | load_status=skipped:proxy visual | validation_status=not_run | upload_status=not_run | draw_status=not_run")
          .arg(visual.frame_id));
      continue;
    }
    if (visual.type != RobotGeometryType::Mesh) {
      continue;
    }
    const QString skip_reason = opengl_candidate_skip_reason(visual);
    if (!skip_reason.isEmpty()) {
      const QString resolved_text = visual.mesh_resolved_path.trimmed().isEmpty() ?
        QString("<empty>") : visual.mesh_resolved_path;
      QString file_access = "not_run";
      if (!visual.mesh_resolved_path.trimmed().isEmpty()) {
        file_access = file_access_diagnostics_text(visual.mesh_filename, visual.mesh_resolved_path);
      }
      rows.push_back(
        QString("frame_id=%1 | uri=%2 | resolved_path=%3 | path_probe_status={%4} | accepted_for_opengl=false | rejected_as_candidate_reason=%5 | load_status=%6 | validation_status=not_run | upload_status=not_run | draw_status=not_run")
          .arg(visual.frame_id)
          .arg(visual.mesh_filename.isEmpty() ? QString("<empty>") : visual.mesh_filename)
          .arg(resolved_text)
          .arg(file_access)
          .arg(skip_reason)
          .arg(visual.mesh_resolved_path.trimmed().isEmpty() ? QString("unresolved:%1").arg(skip_reason) : QString("skipped:%1").arg(skip_reason)));
      const QString skipped_key = QString("mesh_skipped:%1:%2:%3")
        .arg(visual.frame_id, visual.mesh_filename, skip_reason);
      if (!visual_event_cache_.contains(skipped_key)) {
        visual_event_cache_.insert(skipped_key);
        qCInfo(amrVizMeshLog).noquote() <<
          QString("OpenGL mesh candidate skipped: frame=%1, uri=%2, reason=%3")
            .arg(visual.frame_id)
            .arg(visual.mesh_filename.isEmpty() ? QString("<empty>") : visual.mesh_filename)
            .arg(skip_reason);
      }
      continue;
    }
    const QFileInfo file_info(visual.mesh_resolved_path);
    auto probe_text_it = file_probe_text_cache_.find(visual.mesh_resolved_path);
    if (probe_text_it == file_probe_text_cache_.end()) {
      probe_text_it = file_probe_text_cache_.emplace(
        visual.mesh_resolved_path,
        file_access_diagnostics_text(visual.mesh_filename, visual.mesh_resolved_path)).first;
    }
    const QString file_access = probe_text_it->second;
    QString load_status = visual.mesh_resolved_path.isEmpty() ? "unresolved" : "pending";
    QString validation_status = "pending";
    QString upload_status = "pending";
    QString draw_status = "pending";
    const auto it = mesh_cache_.find(visual.mesh_resolved_path);
    if (it != mesh_cache_.end() && it->second) {
      load_status = it->second->load_status;
      validation_status = it->second->validation_status;
      upload_status = it->second->upload_status;
      draw_status = it->second->draw_status;
    }
    rows.push_back(
      QString("frame_id=%1 | uri=%2 | resolved_path=%3 | resolved_path_length=%4 | resolved_path_utf8_hex=%5 | resolved_path_escaped='%6' | qfileinfo_absolute=%7 | qfileinfo_canonical=%8 | exists=%9 | isFile=%10 | readable=%11 | file_size=%12 | qfile_probe={%13} | scale=%14 %15 %16 | pose=(%17,%18,%19,%20,%21,%22) | accepted_for_opengl=true | load_status=%23 | validation_status=%24 | upload_status=%25 | draw_status=%26")
        .arg(visual.frame_id)
        .arg(visual.mesh_filename)
        .arg(visual.mesh_resolved_path.isEmpty() ? QString("<unresolved>") : visual.mesh_resolved_path)
        .arg(visual.mesh_resolved_path.size())
        .arg(utf8_hex_dump(visual.mesh_resolved_path))
        .arg(escaped_path_text(visual.mesh_resolved_path))
        .arg(file_info.absoluteFilePath())
        .arg(file_info.canonicalFilePath().isEmpty() ? QString("<empty>") : file_info.canonicalFilePath())
        .arg(file_info.exists() ? "true" : "false")
        .arg(file_info.isFile() ? "true" : "false")
        .arg(file_info.isReadable() ? "true" : "false")
        .arg(file_info.exists() ? file_info.size() : 0)
        .arg(file_access)
        .arg(visual.mesh_scale_x)
        .arg(visual.mesh_scale_y)
        .arg(visual.mesh_scale_z)
        .arg(visual.pose.x)
        .arg(visual.pose.y)
        .arg(visual.pose.z)
        .arg(visual.pose.roll)
        .arg(visual.pose.pitch)
        .arg(visual.pose.yaw)
        .arg(load_status)
        .arg(validation_status)
        .arg(upload_status)
        .arg(draw_status));
  }

  const QString summary = rows.join("\n");
  if (summary == last_status_table_summary_) {
    return;
  }
  last_status_table_summary_ = summary;
  for (const auto &row : rows) {
    qCInfo(amrVizMeshLog).noquote() << QString("OpenGL mesh status: %1").arg(row);
  }
}

void RobotOpenGLWidget::emitOpenGLMeshSummary(const QString &reason)
{
  const int loaded = loadedMeshCount();
  const int rejected = rejectedMeshCount();
  const int uploaded = uploadedMeshCount();
  const QString summary = QString("%1:%2:%3:%4:%5:%6:%7:%8:%9:%10:%11:%12:%13:%14:%15")
    .arg(reason)
    .arg(last_received_visual_count_)
    .arg(last_received_urdf_mesh_visual_count_)
    .arg(last_received_proxy_visual_count_)
    .arg(last_resolved_mesh_visual_count_)
    .arg(visuals_.size())
    .arg(loaded)
    .arg(rejected)
    .arg(uploaded)
    .arg(last_draw_calls_)
    .arg(last_stl_draw_calls_)
    .arg(last_fallback_cube_draw_calls_)
    .arg(last_rendered_stl_triangles_)
    .arg(last_rendered_triangles_)
    .arg(static_cast<unsigned int>(last_draw_error_code_));
  if (summary == last_opengl_mesh_summary_) {
    return;
  }
  last_opengl_mesh_summary_ = summary;
  qCInfo(amrVizOpenGLLog).noquote() <<
    QString("OpenGL mesh summary: total_visuals=%1, urdf_mesh_visuals=%2, proxy_visuals=%3, accepted_opengl_meshes=%4, resolved_meshes=%5, loaded_meshes=%6, validation_rejected_meshes=%7, uploaded_meshes=%8, draw_calls=%9, stl_draw_calls=%10, fallback_cube_draw_calls=%11, rendered_stl_triangles=%12, rendered_triangles=%13, gl_error=%14")
      .arg(last_received_visual_count_)
      .arg(last_received_urdf_mesh_visual_count_)
      .arg(last_received_proxy_visual_count_)
      .arg(visuals_.size())
      .arg(last_resolved_mesh_visual_count_)
      .arg(loaded)
      .arg(rejected)
      .arg(uploaded)
      .arg(last_draw_calls_)
      .arg(last_stl_draw_calls_)
      .arg(last_fallback_cube_draw_calls_)
      .arg(last_rendered_stl_triangles_)
      .arg(last_rendered_triangles_)
      .arg(static_cast<unsigned int>(last_draw_error_code_));
}

void RobotOpenGLWidget::emitPaintDiagnostics(
  const int draw_calls,
  const int stl_draw_calls,
  const int fallback_cube_draw_calls,
  const int debug_axis_draw_count,
  const int debug_cube_draw_count,
  const int rendered_stl_triangles,
  const int rendered_triangles,
  const GLenum error_code)
{
  int uploaded_mesh_count = 0;
  for (const auto &entry : mesh_cache_) {
    if (entry.second && entry.second->uploaded) {
      ++uploaded_mesh_count;
    }
  }
  const QString summary = QString("%1:%2:%3:%4:%5:%6:%7:%8:%9:%10:%11:%12:%13:%14")
    .arg(isVisible() ? "1" : "0")
    .arg(width())
    .arg(height())
    .arg(visuals_.size())
    .arg(static_cast<int>(mesh_cache_.size()))
    .arg(uploaded_mesh_count)
    .arg(draw_calls)
    .arg(stl_draw_calls)
    .arg(fallback_cube_draw_calls)
    .arg(debug_axis_draw_count)
    .arg(debug_cube_draw_count)
    .arg(rendered_stl_triangles)
    .arg(rendered_triangles)
    .arg(static_cast<unsigned int>(error_code));
  const bool may_emit = !paint_diagnostic_timer_.isValid() || paint_diagnostic_timer_.elapsed() >= 1000;
  if (summary == last_paint_summary_ || !may_emit) {
    return;
  }
  last_paint_summary_ = summary;
  paint_diagnostic_timer_.restart();
  qCInfo(amrVizOpenGLLog).noquote() <<
    QString("OpenGL paintGL diagnostics: visible=%1, size=%2x%3, visuals=%4, mesh cache=%5, uploaded mesh count=%6, draw calls=%7, stl_draw_calls=%8, fallback_cube_draw_calls=%9, debug axes draw count=%10, debug cube draw count=%11, rendered_stl_triangles=%12, rendered triangle count=%13, gl_error=%14, focal=(%15,%16,%17), yaw=%18, pitch=%19, distance=%20, pixels_per_meter=%21, debug_camera=%22, debug_axes=%23, debug_cube=%24, force_visible=%25")
      .arg(isVisible() ? "true" : "false")
      .arg(width())
      .arg(height())
      .arg(visuals_.size())
      .arg(static_cast<int>(mesh_cache_.size()))
      .arg(uploaded_mesh_count)
      .arg(draw_calls)
      .arg(stl_draw_calls)
      .arg(fallback_cube_draw_calls)
      .arg(debug_axis_draw_count)
      .arg(debug_cube_draw_count)
      .arg(rendered_stl_triangles)
      .arg(rendered_triangles)
      .arg(static_cast<unsigned int>(error_code))
      .arg(effectiveFocalPoint().x())
      .arg(effectiveFocalPoint().y())
      .arg(effectiveFocalPoint().z())
      .arg(camera_yaw_)
      .arg(camera_pitch_)
      .arg(effectiveCameraDistance())
      .arg(effectivePixelsPerMeter())
      .arg(debugCameraEnabled() ? "true" : "false")
      .arg(debugAxesEnabled() ? "true" : "false")
      .arg(debugCubeEnabled() ? "true" : "false")
      .arg(forceVisibleEnabled() ? "true" : "false");
}

void RobotOpenGLWidget::emitFallbackOnce(const QString &reason)
{
  opengl_failed_ = true;
  opengl_failure_reason_ = reason;
  if (fallback_event_emitted_) {
    return;
  }
  fallback_event_emitted_ = true;
  qCWarning(amrVizOpenGLLog).noquote() <<
    QString("OpenGL robot renderer failed: %1; falling back to proxy backend").arg(reason);
  Q_EMIT visualizationEvent(
    QString("OpenGL robot renderer failed: %1; falling back to proxy backend").arg(reason));
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
    file_probe_text_cache_.erase(it->first);
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
  destroyMeshBuffers(debug_cube_mesh_);
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

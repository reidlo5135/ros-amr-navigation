#ifndef AMR_VISUALIZATION__ROBOT_OPEN_GL_WIDGET_HPP_
#define AMR_VISUALIZATION__ROBOT_OPEN_GL_WIDGET_HPP_

#include <QElapsedTimer>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QSet>
#include <QString>
#include <QVector>
#include <QVector3D>

#include <map>
#include <memory>

#include "amr_visualization/operator_state.hpp"

namespace amr::visualization
{

class RobotOpenGLWidget final : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT

public:
  explicit RobotOpenGLWidget(QWidget *parent = nullptr);
  ~RobotOpenGLWidget() override;

  void setRobotVisuals(const QVector<RobotVisual> &visuals);
  void setCamera(
    const QVector3D &focal_point,
    double yaw,
    double pitch,
    double distance,
    double pixels_per_meter);
  bool hasRenderableMesh(const RobotVisual &visual) const;
  bool shouldSuppressProxyForVisual(const RobotVisual &visual) const;
  bool hasRenderableVisuals() const;
  QString meshStatus(const RobotVisual &visual) const;
  int loadedMeshCount() const;
  int rejectedMeshCount() const;
  void setRenderVisible(bool visible);

Q_SIGNALS:
  void visualizationEvent(const QString &event);

protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;

private:
  struct GpuMesh
  {
    bool attempted{false};
    bool rejected{false};
    bool uploaded{false};
    QVector<float> vertices;
    QVector<unsigned int> indices;
    QOpenGLBuffer vertex_buffer{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer index_buffer{QOpenGLBuffer::IndexBuffer};
    QOpenGLVertexArrayObject vao;
    QString error;
    QString source_path;
    QVector3D raw_min_bounds;
    QVector3D raw_max_bounds;
    QVector3D raw_extent;
    double raw_diagonal{0.0};
    QVector3D min_bounds;
    QVector3D max_bounds;
    QVector3D extent;
    double diagonal{0.0};
    qint64 file_size_bytes{0};
    QString stl_format{"unknown"};
    QString load_status{"pending"};
    QString validation_status{"pending"};
    QString upload_status{"pending"};
    QString draw_status{"pending"};
    bool validation_finite_coordinates{true};
    bool validation_coordinate_in_range{true};
    bool validation_scale_applied{false};
    bool model_matrix_applies_scale{true};
    double validation_max_abs_coordinate{0.0};
    double validation_max_extent{0.0};
    quint32 source_triangle_count{0};
    quint32 loaded_triangle_count{0};
    int draw_call_count{0};
    bool upload_success_repaint_requested{false};
    GLenum upload_error_code{GL_NO_ERROR};
    GLenum draw_error_code{GL_NO_ERROR};
  };

  bool debugAxesEnabled() const;
  bool debugCubeEnabled() const;
  bool forceVisibleEnabled() const;
  bool stlOnlyDebugEnabled() const;
  bool debugMeshBboxEnabled() const;
  bool debugCameraEnabled() const;
  bool verboseDiagnosticsEnabled() const;
  int uploadedMeshCount() const;
  bool updateRuntimeOptions(const QVector<RobotVisual> &visuals);
  void refreshEffectiveTargetFps();
  bool hasDirtyRenderState() const;
  void requestRepaint();
  void requestRepaintIfDirty();
  bool shouldEmitVerboseDiagnostics();
  bool cameraChangedMeaningfully(
    const QVector3D &focal_point,
    double yaw,
    double pitch,
    double distance,
    double pixels_per_meter) const;
  bool visualsChangedMeaningfully(const QVector<RobotVisual> &visuals) const;
  QString openglCandidateSkipReason(const RobotVisual &visual);
  bool isAcceptedOpenGLMeshVisual(const RobotVisual &visual);
  QVector3D cameraRight() const;
  QVector3D cameraForward() const;
  QVector3D cameraUp() const;
  QVector3D effectiveFocalPoint() const;
  double effectiveCameraDistance() const;
  double effectivePixelsPerMeter() const;
  QMatrix4x4 poseMatrixForVisual(const RobotVisual &visual) const;
  QMatrix4x4 modelMatrixForVisual(const RobotVisual &visual) const;
  QMatrix4x4 viewMatrix() const;
  QMatrix4x4 projectionMatrix() const;
  bool loadStlMesh(const RobotVisual &visual, GpuMesh &mesh);
  bool validateMesh(const RobotVisual &visual, GpuMesh &mesh);
  bool uploadMesh(GpuMesh &mesh);
  bool uploadDebugCube();
  void drawMesh(GpuMesh &mesh, const QMatrix4x4 &model, const QVector3D &color);
  void drawMeshBoundingBox(const RobotVisual &visual, const GpuMesh &mesh, int &draw_calls, int &rendered_triangles);
  void drawStlOnlyFallbackCube(
    int &draw_calls,
    int &fallback_cube_draw_calls,
    int &rendered_triangles);
  void drawDebugGeometryForVisual(
    const RobotVisual &visual,
    int &axis_draw_count,
    int &cube_draw_count,
    int &rendered_triangles);
  void emitWidgetCreatedOnce();
  void emitWidgetGeometry(const QString &reason);
  void emitSetVisualsDiagnostics(const QVector<RobotVisual> &incoming, const QSet<QString> &active_paths);
  void emitStlLoadDiagnostics(const RobotVisual &visual, const GpuMesh &mesh);
  void emitUploadDiagnostics(const GpuMesh &mesh, GLenum error_code);
  void emitMeshStatusTable();
  void emitOpenGLMeshSummary(const QString &reason);
  void emitPaintDiagnostics(
    int draw_calls,
    int stl_draw_calls,
    int fallback_cube_draw_calls,
    int debug_axis_draw_count,
    int debug_cube_draw_count,
    int rendered_stl_triangles,
    int rendered_triangles,
    GLenum error_code);
  void emitMeshVisualDiagnostics();
  void emitFallbackOnce(const QString &reason);
  void pruneInactiveMeshes(const QSet<QString> &active_paths);
  void destroyMeshBuffers(GpuMesh &mesh);
  void destroyAllMeshBuffers();
  QString meshBoundsText(const GpuMesh &mesh) const;

  QVector<RobotVisual> visuals_;
  QVector<RobotVisual> incoming_visuals_;
  std::map<QString, std::unique_ptr<GpuMesh>> mesh_cache_;
  std::map<QString, QString> file_probe_text_cache_;
  std::map<QString, QString> candidate_skip_reason_cache_;
  QSet<QString> warning_cache_;
  QSet<QString> success_cache_;
  QSet<QString> visual_event_cache_;
  QSet<QString> render_event_cache_;
  QSet<QString> stl_load_event_cache_;
  QSet<QString> upload_event_cache_;
  QOpenGLShaderProgram program_;
  GpuMesh debug_cube_mesh_;
  bool initialized_{false};
  bool opengl_failed_{false};
  bool fallback_event_emitted_{false};
  bool widget_created_event_emitted_{false};
  bool initialize_entered_event_emitted_{false};
  bool paint_entered_event_emitted_{false};
  bool last_render_visible_state_{false};
  QString opengl_failure_reason_;
  QString last_mesh_summary_;
  QString last_set_visuals_summary_;
  QString last_paint_summary_;
  QString last_status_table_summary_;
  QString last_opengl_mesh_summary_;
  QString last_visible_geometry_;
  QString last_stl_fallback_reason_;
  QElapsedTimer paint_diagnostic_timer_;
  QElapsedTimer repaint_throttle_timer_;
  QElapsedTimer paint_fps_timer_;
  int last_draw_calls_{0};
  int last_stl_draw_calls_{0};
  int last_fallback_cube_draw_calls_{0};
  int last_rendered_triangles_{0};
  int last_rendered_stl_triangles_{0};
  GLenum last_draw_error_code_{GL_NO_ERROR};
  int last_received_visual_count_{0};
  int last_received_urdf_mesh_visual_count_{0};
  int last_received_proxy_visual_count_{0};
  int last_unresolved_pose_mesh_visual_count_{0};
  int last_resolved_mesh_visual_count_{0};
  bool verbose_diagnostics_{false};
  bool auto_software_profile_{true};
  bool software_renderer_detected_{false};
  bool renderer_profile_known_{false};
  bool camera_dirty_{true};
  bool robot_pose_dirty_{true};
  bool mesh_upload_dirty_{true};
  bool resize_dirty_{true};
  bool debug_dirty_{false};
  bool repaint_queued_{false};
  bool software_renderer_warning_emitted_{false};
  int explicit_target_fps_{0};
  int software_target_fps_{8};
  int hardware_target_fps_{30};
  int target_fps_{30};
  int skipped_repaint_count_{0};
  int paint_sample_count_{0};
  double actual_paint_fps_{0.0};
  qint64 last_paint_elapsed_ms_{0};
  qint64 last_set_visuals_elapsed_ms_{0};
  GLenum last_reported_draw_error_code_{GL_NO_ERROR};
  double pose_epsilon_m_{0.003};
  double yaw_epsilon_rad_{0.003};
  QVector3D focal_point_{0.0F, 0.0F, 0.0F};
  double camera_yaw_{0.0};
  double camera_pitch_{1.5707963267948966};
  double camera_distance_{8.0};
  double pixels_per_meter_{90.0};
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__ROBOT_OPEN_GL_WIDGET_HPP_

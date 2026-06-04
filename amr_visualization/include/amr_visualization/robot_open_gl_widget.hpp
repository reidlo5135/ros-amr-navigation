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
  bool hasRenderableVisuals() const;

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
    QVector3D min_bounds;
    QVector3D max_bounds;
    QVector3D extent;
    double diagonal{0.0};
  };

  QVector3D cameraRight() const;
  QVector3D cameraForward() const;
  QVector3D cameraUp() const;
  QMatrix4x4 modelMatrixForVisual(const RobotVisual &visual) const;
  QMatrix4x4 viewMatrix() const;
  QMatrix4x4 projectionMatrix() const;
  bool loadStlMesh(const RobotVisual &visual, GpuMesh &mesh);
  bool validateMesh(const RobotVisual &visual, GpuMesh &mesh);
  bool uploadMesh(GpuMesh &mesh);
  void pruneInactiveMeshes(const QSet<QString> &active_paths);
  void destroyMeshBuffers(GpuMesh &mesh);
  void destroyAllMeshBuffers();
  QString meshBoundsText(const GpuMesh &mesh) const;

  QVector<RobotVisual> visuals_;
  std::map<QString, std::unique_ptr<GpuMesh>> mesh_cache_;
  QSet<QString> warning_cache_;
  QSet<QString> success_cache_;
  QSet<QString> render_event_cache_;
  QOpenGLShaderProgram program_;
  bool initialized_{false};
  QVector3D focal_point_{0.0F, 0.0F, 0.0F};
  double camera_yaw_{0.0};
  double camera_pitch_{1.5707963267948966};
  double camera_distance_{8.0};
  double pixels_per_meter_{90.0};
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__ROBOT_OPEN_GL_WIDGET_HPP_

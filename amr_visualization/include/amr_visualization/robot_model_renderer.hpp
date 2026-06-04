#ifndef AMR_VISUALIZATION__ROBOT_MODEL_RENDERER_HPP_
#define AMR_VISUALIZATION__ROBOT_MODEL_RENDERER_HPP_

#include <QString>

#include <memory>

namespace amr::visualization
{

class RobotModelRenderer
{
public:
  virtual ~RobotModelRenderer() = default;

  virtual QString backendName() const = 0;
  virtual QString meshRenderMode() const = 0;
  virtual bool loadsMeshFiles() const = 0;
  virtual QString statusMessage() const = 0;

  static std::unique_ptr<RobotModelRenderer> create(const QString &backend_name);
};

class ProxyRobotRenderer final : public RobotModelRenderer
{
public:
  QString backendName() const override;
  QString meshRenderMode() const override;
  bool loadsMeshFiles() const override;
  QString statusMessage() const override;
};

class QPainterWireframeRobotRenderer final : public RobotModelRenderer
{
public:
  QString backendName() const override;
  QString meshRenderMode() const override;
  bool loadsMeshFiles() const override;
  QString statusMessage() const override;
};

class OpenGLRobotRenderer final : public RobotModelRenderer
{
public:
  QString backendName() const override;
  QString meshRenderMode() const override;
  bool loadsMeshFiles() const override;
  QString statusMessage() const override;
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__ROBOT_MODEL_RENDERER_HPP_

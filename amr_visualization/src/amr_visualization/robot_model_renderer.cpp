#include "amr_visualization/robot_model_renderer.hpp"

namespace amr::visualization
{

std::unique_ptr<RobotModelRenderer> RobotModelRenderer::create(const QString &backend_name)
{
  const QString normalized = backend_name.trimmed().toLower();
  if (normalized == "qpainter_wireframe") {
    return std::make_unique<QPainterWireframeRobotRenderer>();
  }
  if (normalized == "opengl") {
    return std::make_unique<OpenGLRobotRenderer>();
  }
  return std::make_unique<ProxyRobotRenderer>();
}

QString ProxyRobotRenderer::backendName() const
{
  return "proxy";
}

QString ProxyRobotRenderer::meshRenderMode() const
{
  return "proxy";
}

bool ProxyRobotRenderer::loadsMeshFiles() const
{
  return false;
}

QString ProxyRobotRenderer::statusMessage() const
{
  return "Robot renderer backend: proxy; Mesh backend disabled: rendering proxy";
}

QString QPainterWireframeRobotRenderer::backendName() const
{
  return "qpainter_wireframe";
}

QString QPainterWireframeRobotRenderer::meshRenderMode() const
{
  return "wireframe";
}

bool QPainterWireframeRobotRenderer::loadsMeshFiles() const
{
  return true;
}

QString QPainterWireframeRobotRenderer::statusMessage() const
{
  return "Robot renderer backend: qpainter_wireframe; Mesh backend enabled: rendering wireframe";
}

QString OpenGLRobotRenderer::backendName() const
{
  return "opengl";
}

QString OpenGLRobotRenderer::meshRenderMode() const
{
  return "opengl";
}

bool OpenGLRobotRenderer::loadsMeshFiles() const
{
  return true;
}

QString OpenGLRobotRenderer::statusMessage() const
{
  return "Robot renderer backend: opengl; Mesh backend enabled: rendering shaded solid OpenGL";
}

}  // namespace amr::visualization

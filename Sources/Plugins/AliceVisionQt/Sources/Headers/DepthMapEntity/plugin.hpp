#pragma once

#include <DepthMapEntity/DepthMapEntity.hpp>

#include <QtQml/QQmlExtensionPlugin>
#include <QtQml/QtQml>

namespace depthMapEntity {

class DepthMapEntityPlugin : public QQmlExtensionPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "DepthMapEntity.qmlPlugin")

  public:
    void initializeEngine(QQmlEngine* engine, const char* uri) override
    {
        // Fix "unused parameter" warnings; should be replaced by [[maybe_unused]] when C++17 is supported
        (void)engine;
        (void)uri;
    }
    void registerTypes(const char* uri) override
    {
        Q_ASSERT(uri == QLatin1String("DepthMapEntity"));
        qmlRegisterType<DepthMapEntity>(uri, 2, 1, "DepthMapEntity");
    }
};
}  // namespace depthMapEntity

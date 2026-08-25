#pragma once

/// aerovistaSync 门面层的公开边界类型：纯 POD + 注入接口，零 vsg 依赖。
/// 作用：让消费方（含完全无 vsg 的 viewhost）在编译/链接期都不接触 vsg 头文件。
/// 内部实现可复用 vsg header-only 数学（见 SynchronSystem.cpp），但不在公开头暴露。
namespace aerovista::sync
{
    /// 纯 POD 三维向量（公开边界类型，替代公开签名里的 vsg::dvec3）。
    struct DVec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

} // namespace aerovista::sync

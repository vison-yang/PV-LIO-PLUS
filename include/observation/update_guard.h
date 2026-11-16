/** @file update_guard.h @brief 新观测路径失效后的单帧状态和协方差回滚。 */
#pragma once

#include <utility>

namespace pv_lio_plus::observation
{
/** @brief 拒绝未支持的少行分支、维度错配及非正信息，检查对象即生产观测行。 */
template <typename Measurement>
bool ValidScalarRows(const Measurement &data, int minRows)
{
    return data.valid && minRows > 0 && data.h.size() >= minRows && data.h_x.rows() == data.h.size() &&
           data.h_x.cols() == 12 && data.R.size() == data.h.size() && data.h_x.allFinite() && data.h.allFinite() &&
           data.R.allFinite() && (data.R.array() > 0.0).all();
}

/** @brief 任意一次线性化拒绝后恢复预测；回调只能置拒绝标志，不能在同帧清除。 */
template <typename Filter, typename Update>
bool RunGuardedUpdate(Filter &filter, bool &isRejected, Update &&update)
{
    auto predictedState = filter.get_x();
    auto predictedCov   = filter.get_P();
    isRejected          = false;
    try
    {
        const bool isConverged = std::forward<Update>(update)();
        if (!isRejected) return isConverged;
    } catch (...)
    {
        filter.change_x(predictedState);
        filter.change_P(predictedCov);
        throw;
    }
    filter.change_x(predictedState);
    filter.change_P(predictedCov);
    return false;
}
}  // namespace pv_lio_plus::observation

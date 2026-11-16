/** @file weight_config.h @brief 独立权重组件的启动配置约束。 */
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace pv_lio_plus::observation
{
/** @brief 只验证已开启组件的参数；禁止把未支持组合静默当作后端默认值。 */
inline void ValidateWeightConfig(const std::string &model, const std::string &backend, double scaleM)
{
    if (model == "backend_default") return;
    if (model != "geman_mcclure") throw std::invalid_argument("unknown observation/weight_model: " + model);
    if (backend != "ikdtree" && backend != "ivox" && backend != "i_octree" && backend != "ig_lio_plane")
        throw std::invalid_argument("GM requires ikdtree/ivox/i_octree/ig_lio_plane");
    if (!std::isfinite(scaleM) || scaleM <= 0.0)
        throw std::invalid_argument("observation/gm_scale_m must be positive and finite");
}
}  // namespace pv_lio_plus::observation

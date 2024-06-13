/**
 * @file velocity_limit_filter.hpp
 * @author Takuma Nakao
 * @brief 速度制限フィルタ
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <algorithm>
#include <tuple>
#include <optional>

namespace tlab
{

namespace internal
{
class DiffPair {
public:
    DiffPair(double Ts_, double gpd_) : Ts(Ts_), gpd(gpd_) {}

    void reset() { reset(0.0f); }

    void reset(double u)
    {
        u1 = u;
        y1diff = 0;
        y1pass = 0;
    }

    std::tuple<double, double> calcu(double u)
    {
        double diff = (2.0 * gpd * (u - u1) + (2.0 - Ts * gpd) * y1diff) / (2.0 + Ts * gpd);
        double pass = (gpd * Ts * (u + u1) + (2.0 - gpd * Ts) * y1pass) / (2.0 + Ts * gpd);

        u1 = u;
        y1diff = diff;
        y1pass = pass;

        return {diff, pass};
    }

private:
    double Ts, gpd;
    double u1 = 0, y1diff = 0, y1pass = 0;
};

class Intr {
public:
    Intr(double Ts_) : Ts(Ts_) {}

    void reset(double u = 0)
    {
        val = u;
        u1 = 0;
    }

    double calcu(double u)
    {
        val += (u1 + u) * Ts * 0.5;
        u1 = u;
        return val;
    }

private:
    double Ts;
    double val, u1 = 0;
};
} // namespace internal

class VelocityLimitFilter {
public:
    VelocityLimitFilter(double v_max_, double Ts_) : VelocityLimitFilter(v_max_, Ts_, 1.0 / Ts_, 0.5 / Ts_) {}

    VelocityLimitFilter(double v_max_, double Ts_, std::pair<double, double> limit) : VelocityLimitFilter(v_max_, Ts_, 1.0 / Ts_, 0.5 / Ts_, limit) {}

    VelocityLimitFilter(double v_max_, double Ts_, double gpd_, double fb_gain, std::optional<std::pair<double, double>> limit = std::nullopt) : Ts(Ts_), v_max(v_max_), gpd(gpd_), Cfb(fb_gain), dp(Ts, gpd), intr(Ts) { reset(); }

    void reset() { reset(0); }
    void reset(double u)
    {
        u1 = u;
        y1 = u;
        dp.reset(u);
        intr.reset(u);
    }

    double filtering(double u)
    {
        if (limit_) {
            u = std::clamp(u, limit_.value().first, limit_.value().second);
        }
        auto [diff, pass] = dp.calcu(u);
        double v = diff - (Cfb * (y1 - pass));
        v = std::clamp(v, -v_max, v_max);
        double y = intr.calcu(v);
        if (limit_) {
            y = std::clamp(y, limit_.value().first, limit_.value().second);
        }

        u1 = u;
        y1 = y;
        return y;
    }

private:
    double Ts, v_max, gpd, Cfb;
    internal::DiffPair dp;
    internal::Intr intr;
    double u1 = 0, y1 = 0;

    std::optional<std::pair<double, double>> limit_;
};

class StateHoledVelocityLimitFilter : public VelocityLimitFilter {
public:
    using VelocityLimitFilter::VelocityLimitFilter;
    void set_input(double u_) { u = u_; }

    double filtering()
    {
        y = VelocityLimitFilter::filtering(u);
        return y;
    }

    void reset(double u_)
    {
        u = u_;
        y = u_;
        VelocityLimitFilter::reset(u_);
    }

    double u = 0, y = 0;
};

class ClampVelLimitFilter : public StateHoledVelocityLimitFilter {
public:
    double min;
    double max;
    ClampVelLimitFilter(double min_, double max_, double vmax, double ts) : StateHoledVelocityLimitFilter(vmax, ts), min(min_), max(max_)
    {
        u = min;
        y = min;
    }
    void set_input(double u_) { u = std::clamp(u_, min, max); }
    void reset(void) { reset(min); }
    void reset(double u_)
    {
        StateHoledVelocityLimitFilter::reset(std::clamp(u_, min, max));
        u = std::clamp(u_, min, max);
        y = std::clamp(u_, min, max);
    }
};

} // namespace tlab
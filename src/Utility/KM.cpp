#include "Common.hpp"
#include "Utility.hpp"

// Notation: 1)m >= n,otherwise it will stick in an infinite loop.
//          2)res is the result of m match n, if on one match m, the res[m] is (2^32 - 1).
// TODO: fixme
std::vector<uint32_t> solveKM(const uint32_t n, const uint32_t m, const std::vector<double>& w) {
    const auto size = std::max(n, m) + 1;
    std::vector<double> lh(size), rh(size), slack(size);
    std::vector<uint32_t> pair(size), pre(size);
    std::vector<bool> flag(size);

    const auto reset = [](auto& c, auto value) { std::fill(c.begin() + 1, c.end(), value); };

    const auto aug = [&](const uint32_t s) {
        reset(flag, false);
        reset(pre, 0);
        reset(slack, 1e9);
        pair[0] = s;
        uint32_t u = 0;
        do {
            uint32_t v = pair[u], nxt = 0;
            double minh = 1e9;
            flag[u] = true;
            for(uint32_t i = 1; i <= m; ++i)
                if(!flag[i]) {
                    const auto delta = lh[v] + rh[i] - w[(v - 1) * m + i - 1];
                    if(delta < slack[i])
                        slack[i] = delta, pre[i] = u;
                    if(minh > slack[i])
                        minh = slack[i], nxt = i;
                }
            for(uint32_t i = 0; i <= m; ++i)
                if(flag[i])
                    lh[pair[i]] -= minh, rh[i] += minh;
                else
                    slack[i] -= minh;
            u = nxt;
        } while(pair[u]);
        while(u) {
            int p = pre[u];
            pair[u] = pair[p];
            u = p;
        }
    };

    for(uint32_t i = 1; i <= n; ++i) {
        double maxh = 0;
        for(uint32_t j = 1; j <= m; ++j)
            maxh = std::fmax(maxh, w[(i - 1) * m + j - 1]);
        lh[i] = maxh;
    }
    reset(rh, 0.0);
    reset(pair, 0);
    for(uint32_t i = 1; i <= n; ++i)
        aug(i);
    std::vector<uint32_t> res(m);
    for(uint32_t idx = 0; idx < m; ++idx)
        res[idx] = pair[idx + 1] - 1;

    return res;
}

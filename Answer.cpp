#include "HPCAnswerInclude.hpp"

// #undef LOCAL
#ifdef LOCAL
#include "local.h"
using namespace std;
namespace std
{
ostream& operator<<(ostream& os, const hpc::Vec2& p)
{
    char buf[128];
    sprintf(buf, "(%7.3f, %7.3f)", p.x, p.y);
    os << buf;
    return os;
}
}
#else

#define assert(a)
#define abort()
#define debug(a)
#define dump(a)

#endif

#define rep(i, n) for (int i = 0; i < (int)(n); ++i)
#define erep(i, n) for (int i = 0; i <= (int)(n); ++i)

typedef unsigned int uint;
typedef unsigned short ushort;
typedef unsigned char uchar;

uint pss(uint first, uint second) { return (first << 16) | second; }
ushort pcc(uchar first, uchar second) { return (first << 8) | second; }

ushort pss_first(uint pair) { return pair >> 16; }
ushort pss_second(uint pair) { return pair & 0xffff; }
uchar pcc_first(ushort pair) { return pair >> 8; }
uchar pcc_second(ushort pair) { return pair & 0xff; }

using namespace hpc;

namespace solver
{
template <typename T>
inline
void swap(T& a, T& b)
{
    T c = a;
    a = b;
    b = c;
}

template <typename T>
inline
T min(const T& a, const T& b)
{
    return a < b ? a : b;
}

template <typename T>
inline
T max(const T& a, const T& b)
{
    return a > b ? a : b;
}

template <typename T>
inline
T abs(const T& n) { return n >= 0 ? n : -n; }


// Vec2 decel_vel(const Vec2& vel)
// {
//     const float speed = max<float>(0, vel.length() - Parameter::CharaDecelSpeed());
//     if (speed > 0)
//         return vel.getNormalized(speed);
//     else
//         return Vec2();
// }
void decel_vel(Vec2& vel)
{
    const float speed = max<float>(0, vel.length() - Parameter::CharaDecelSpeed());
    if (speed > 0)
        vel.normalize(speed);
    else
        vel.reset();
}
void decel_vel(Vec2& vel, int cur_vel_level)
{
    if (cur_vel_level == 0)
        decel_vel(vel);
    else
    {
        static const float coef[] = {0,0.000000,0.500000,0.666667,0.750000,0.800000,0.833333,0.857143,0.875000,0.888889,0.900000,0.909091,0.916667,0.923077,0.928571};
        vel *= coef[cur_vel_level];;
    }
}

const int CharaAccelCountMax = 9; // -pgでコンパイルするために

bool is_equal(const Vec2& a, const Vec2& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return abs(dx) + abs(dy) < 1e-4;
}

const int MAX_SEARCH_TURN = 85;
class ActionStrategy
{
public:
    ActionStrategy()
        : cache_i(0), cache_size(0)
    {
    }

    void reset()
    {
        cache_i = cache_size = 0;
    }

    int size() const { return cache_size; }
    int index() const { return cache_i; }
    bool cached_strategy() const { return cache_i < cache_size; }
    bool is_equal_strategy(const Chara& player)
    {
//         assert(player.accelCount() == get_accel_count());
        return
//             player.accelCount() == get_accel_count() &&
//             player.pos() == get_pos() &&
//             player.vel() == get_vel();
            is_equal(player.pos(), get_pos()) &&
            is_equal(player.vel(), get_vel());
    }

    void next_turn()
    {
        ++cache_i;
    }
    const Vec2& get_pos() const
    {
        assert(0 <= cache_i && cache_i < cache_size);
        return cache_pos[cache_i];
    }
    const Vec2& get_vel() const
    {
        assert(0 <= cache_i && cache_i < cache_size);
        return cache_vel[cache_i];
    }
//     int get_accel_count() const
//     {
//         assert(0 <= cache_i && cache_i < cache_size);
//         return cache_accel_count[cache_i];
//     }

    const Action& get_action() const
    {
        assert(0 <= cache_i && cache_i < cache_size);
        return cache_action[cache_i];
    }

private:
    int cache_i;
    int cache_size;
    Vec2 cache_pos[MAX_SEARCH_TURN];
    Vec2 cache_vel[MAX_SEARCH_TURN];
//     int cache_accel_count[MAX_SEARCH_TURN];
    Action cache_action[MAX_SEARCH_TURN];

public:
    void search(const StageAccessor& stage_accessor, const int search_turns)
    {
        assert(0 <= search_turns && search_turns < MAX_SEARCH_TURN);

        const Chara& player = stage_accessor.player();
        const LotusCollection& lotuses = stage_accessor.lotuses();
        const Field& field = stage_accessor.field();

        const float flow_vel_y = field.flowVel().y;
        const bool flow = flow_vel_y > 0;

        Vec2 target_pos[Parameter::LotusCountMax];
        float collision_squared_dist[Parameter::LotusCountMax];
        rep(target_lotus, lotuses.count())
        {
            const Lotus& lotus = lotuses[target_lotus];
            const Vec2& next = lotus.pos();
            const Vec2& nextnext = lotuses[(target_lotus + 1) % lotuses.count()].pos();
            target_pos[target_lotus] = next + (nextnext - next).getNormalized(lotus.radius());

            const float collision_dist = lotus.radius() + Parameter::CharaRadius();
            collision_squared_dist[target_lotus] = collision_dist * collision_dist - 1e-7;
        }

        const int MAX_VEL_LEVEL = 14; // Parameter::CharaAccelSpeed / Parameter::CharaDecelSpeed
        static Vec2 dp_pos[MAX_SEARCH_TURN][CharaAccelCountMax + 1][MAX_VEL_LEVEL];
        static Vec2 dp_vel[MAX_SEARCH_TURN][CharaAccelCountMax + 1][MAX_VEL_LEVEL];
        static char dp_passed_lotus[MAX_SEARCH_TURN][CharaAccelCountMax + 1][MAX_VEL_LEVEL];
        static ushort dp_prev[MAX_SEARCH_TURN][CharaAccelCountMax + 1][MAX_VEL_LEVEL];
        static Action dp_action[MAX_SEARCH_TURN][CharaAccelCountMax + 1][MAX_VEL_LEVEL];

        erep(dp_i, search_turns) erep(accel_count, CharaAccelCountMax) rep(vel_level, MAX_VEL_LEVEL)
            dp_passed_lotus[dp_i][accel_count][vel_level] = -1;
        dp_pos[0][player.accelCount()][0] = player.pos();
        dp_vel[0][player.accelCount()][0] = player.vel();
        dp_passed_lotus[0][player.accelCount()][0] = player.passedLotusCount();
        dp_prev[0][player.accelCount()][0] = 0;

        const Action WAIT_ACTION = Action::Wait();

        int target_lotus_table[Parameter::LotusCountMax * Parameter::StageRoundCount + 1];
        rep(passed_lotus, lotuses.count() * Parameter::StageRoundCount + 1)
            target_lotus_table[passed_lotus] = passed_lotus % lotuses.count();

        int searching_turn = 0;
        int accel_wait_turn = player.accelWaitTurn();
        for (int dp_i = 0; dp_i < search_turns; ++dp_i)
        {
            const int add_accel_count = --accel_wait_turn == 0;
            if (accel_wait_turn == 0)
                accel_wait_turn = CharaAccelCountMax + 1;

            bool found_goal = false;
            erep(accel_count, CharaAccelCountMax) rep(vel_level, MAX_VEL_LEVEL)
            {
                if (dp_passed_lotus[dp_i][accel_count][vel_level] < 0)
                    continue;

                const int passed_lotus = dp_passed_lotus[dp_i][accel_count][vel_level];
//                 const int target_lotus = passed_lotus % lotuses.count();
                const int target_lotus = target_lotus_table[passed_lotus];
                const Vec2& cur_target_pos = target_pos[target_lotus];

                const Vec2& cur_pos = dp_pos[dp_i][accel_count][vel_level];
                const Vec2& cur_vel = dp_vel[dp_i][accel_count][vel_level];

                // wait
                {
                    const int naccel_count = min(CharaAccelCountMax, accel_count + add_accel_count);
                    const int nvel_level = max(0, vel_level - 1);

                    Vec2 next_pos = cur_vel;
                    next_pos.y += field.flowVel().y;
                    next_pos += cur_pos;

                    const int next_passed_lotus = passed_lotus + (next_pos.squareDist(lotuses[target_lotus].pos()) < collision_squared_dist[target_lotus]);
//                     const int next_target_lotus = next_passed_lotus % lotuses.count();
                    const int next_target_lotus = target_lotus_table[next_passed_lotus];

                    Vec2 next_target_pos = target_pos[next_target_lotus];
                    if (flow)
                        next_target_pos.y -= cur_pos.dist(target_pos[next_target_lotus]) * flow_vel_y;

                    if (
                        (
                             next_passed_lotus > dp_passed_lotus[dp_i + 1][naccel_count][nvel_level] ||
                             (
                              next_passed_lotus == dp_passed_lotus[dp_i + 1][naccel_count][nvel_level] &&
                              next_pos.squareDist(next_target_pos) < dp_pos[dp_i + 1][naccel_count][nvel_level].squareDist(next_target_pos)
                             )
                        )
                       )
                    {
                        assert(0 <= naccel_count && naccel_count <= CharaAccelCountMax);
                        assert(0 <= nvel_level && nvel_level <= MAX_VEL_LEVEL);

                        dp_pos[dp_i + 1][naccel_count][nvel_level] = next_pos;

//                         dp_vel[dp_i + 1][naccel_count][nvel_level] = decel_vel(cur_vel);
                        Vec2 next_vel = cur_vel;
                        decel_vel(next_vel, vel_level);
                        dp_vel[dp_i + 1][naccel_count][nvel_level] = next_vel;

                        dp_passed_lotus[dp_i + 1][naccel_count][nvel_level] = next_passed_lotus;
                        dp_prev[dp_i + 1][naccel_count][nvel_level] = pcc(accel_count, vel_level);
                        dp_action[dp_i + 1][naccel_count][nvel_level] = WAIT_ACTION;

                        if (next_passed_lotus == lotuses.count() * Parameter::StageRoundCount)
                            found_goal = true;
                    }
                }

                // accel
                if (accel_count > 0)
                {
                    const int naccel_count = min(CharaAccelCountMax, accel_count - 1 + add_accel_count);
                    const int nvel_level = MAX_VEL_LEVEL - 1;

//                     Vec2 acceled_vel = (cur_target_pos - cur_pos).getNormalized(Parameter::CharaAccelSpeed());
                    Vec2 acceled_vel = cur_target_pos;
                    acceled_vel -= cur_pos;
                    acceled_vel.normalize(Parameter::CharaAccelSpeed());

                    Vec2 next_pos = acceled_vel;
                    next_pos.y += flow_vel_y;
                    next_pos += cur_pos;

                    const int next_passed_lotus = passed_lotus + (next_pos.squareDist(lotuses[target_lotus].pos()) < collision_squared_dist[target_lotus]);
//                     const int next_target_lotus = next_passed_lotus % lotuses.count();
                    const int next_target_lotus = target_lotus_table[next_passed_lotus];

                    Vec2 next_target_pos = target_pos[next_target_lotus];
                    if (flow)
                        next_target_pos.y -= cur_pos.dist(target_pos[next_target_lotus]) * flow_vel_y;

                    assert(0 <= naccel_count && naccel_count <= CharaAccelCountMax);
                    assert(0 <= nvel_level && nvel_level <= MAX_VEL_LEVEL);

                    if (
                        (
                         next_passed_lotus > dp_passed_lotus[dp_i + 1][naccel_count][nvel_level] ||
                         (
                          next_passed_lotus == dp_passed_lotus[dp_i + 1][naccel_count][nvel_level] &&
                          next_pos.squareDist(next_target_pos) < dp_pos[dp_i + 1][naccel_count][nvel_level].squareDist(next_target_pos)
                         )
                        )
                       )
                    {
                        dp_pos[dp_i + 1][naccel_count][nvel_level] = next_pos;

//                         dp_vel[dp_i + 1][naccel_count][nvel_level] = decel_vel(acceled_vel);
//                         decel_vel(acceled_vel);
                        decel_vel(acceled_vel, MAX_VEL_LEVEL);
                        dp_vel[dp_i + 1][naccel_count][nvel_level] = acceled_vel;

                        dp_passed_lotus[dp_i + 1][naccel_count][nvel_level] = next_passed_lotus;
                        dp_prev[dp_i + 1][naccel_count][nvel_level] = pcc(accel_count, vel_level);
                        dp_action[dp_i + 1][naccel_count][nvel_level] = Action::Accel(cur_target_pos);

                        if (next_passed_lotus == lotuses.count() * Parameter::StageRoundCount)
                            found_goal = true;
                    }
                }
            }

            ++searching_turn;
            if (found_goal)
                break;
        }
        assert(searching_turn <= search_turns);

        int best_passed_lotus = -2;
        float best_sq_dist = -1;
        int best_accel_count = -1, best_vel_level = -1;
        erep(accel_count, CharaAccelCountMax) rep(vel_level, MAX_VEL_LEVEL)
        {
            const int passed_lotus = dp_passed_lotus[searching_turn][accel_count][vel_level];
            if ((
                 passed_lotus > best_passed_lotus ||
                 (
                  passed_lotus == best_passed_lotus &&
                  dp_pos[searching_turn][accel_count][vel_level].squareDist(target_pos[passed_lotus % lotuses.count()]) < best_sq_dist
                 )
                )
               )
            {
                best_passed_lotus = passed_lotus;
                best_sq_dist = dp_pos[searching_turn][accel_count][vel_level].squareDist(target_pos[passed_lotus % lotuses.count()]);
                best_accel_count = accel_count;
                best_vel_level = vel_level;
            }
        }
        assert(best_passed_lotus != -2);

        for (int dp_i = searching_turn, accel_count = best_accel_count, vel_level = best_vel_level; dp_i > 0; --dp_i)
        {
            cache_pos[dp_i - 1] = dp_pos[dp_i][accel_count][vel_level];
            cache_vel[dp_i - 1] = dp_vel[dp_i][accel_count][vel_level];
            cache_action[dp_i - 1] = dp_action[dp_i][accel_count][vel_level];
//             cache_accel_count[dp_i - 1] = accel_count;

            int paccel_count = pcc_first(dp_prev[dp_i][accel_count][vel_level]);
            int pvel_level = pcc_second(dp_prev[dp_i][accel_count][vel_level]);
            assert(0 <= paccel_count && paccel_count <= CharaAccelCountMax);
            assert(0 <= pvel_level && pvel_level <= MAX_VEL_LEVEL);
            accel_count = paccel_count;
            vel_level = pvel_level;
        }

        cache_i = 0;
        cache_size = searching_turn;
    }
};
}

namespace hpc
{
using namespace solver;

int stage_no = -1;
ActionStrategy action_strategy;
int prev;
void Answer::Init(const StageAccessor& aStageAccessor)
{
    ++stage_no;
//     dump(stage_no);
//     if (stage_no > 0)
//         exit(0);
    action_strategy.reset();
    prev = 0;
}

Action Answer::GetNextAction(const StageAccessor& aStageAccessor)
{
    const Chara& player = aStageAccessor.player();

    if (action_strategy.index() + 1 >= action_strategy.size() * 8 / 10 || !action_strategy.is_equal_strategy(player))
    {
//         const int turns = solver::min(MAX_SEARCH_TURN - 1, MAX_SEARCH_TURN / 2 + player.passedTurn());
        int turns = MAX_SEARCH_TURN - 1;
//         if (player.passedTurn() - prev < 5)
//             turns = MAX_SEARCH_TURN / 2;
        action_strategy.search(aStageAccessor, turns);
        prev = player.passedTurn();
    }
    else
        action_strategy.next_turn();

    return action_strategy.get_action();
}
}

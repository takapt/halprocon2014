#include "HPCAnswerInclude.hpp"

#ifdef LOCAL
#include "local.h"
#endif

#define rep(i, n) for (int i = 0; i < (int)(n); ++i)

using namespace hpc;

namespace hpc
{
int st = -1;
    void Answer::Init(const StageAccessor& aStageAccessor)
    {
        ++st;
//         printf("%.3f, %.3f\n", aStageAccessor.field().rect().width(), aStageAccessor.field().rect().height());
    }

    Action Answer::GetNextAction(const StageAccessor& aStageAccessor)
    {
        const Chara& player = aStageAccessor.player();
        const LotusCollection& lotuses = aStageAccessor.lotuses();
//         const Field& field = aStageAccessor.field();

        const Lotus& lotus = lotuses[player.targetLotusNo()];
        const Vec2& next = lotus.pos();
        const Vec2 vec = next - player.pos();
        const Vec2& nextnext = lotuses[(player.targetLotusNo() + 1) % aStageAccessor.lotuses().count()].pos();
        const Vec2 target = next + (nextnext - next).getNormalized(lotus.radius() * 0.9);

//         if (st == 94 && abs(player.passedTurn() - 590) < 6)
//         {
//             dump(player.targetLotusNo());
//             dump(player.accelCount());
//             dump(player.vel().getProjected(vec).length());
//             fprintf(stderr, "(%.3f, %.3f), (%.3f, %.3f)\n", player.vel().x, player.vel().y, vec.x, vec.y);
//             cerr << endl;
//         }

        if (player.accelCount() == 0)
            return Action::Wait();

        if (player.vel().dot(vec) / vec.length() > 0.23)
            return Action::Wait();

        {
            Chara p = player;
            int turns = 15;
            if (vec.dot(aStageAccessor.field().flowVel()) > 0.4)
                turns = 50;
            rep(t, turns)
            {
                if (Collision::IsHit(p.region(), lotus.region()))
                    return Action::Wait();
                p.move();
            }
        }

        if (vec.dot(aStageAccessor.field().flowVel()) < 0)
            return Action::Accel(target - vec.length() * aStageAccessor.field().flowVel());
        else
            return Action::Accel(target - 0.3 * aStageAccessor.field().flowVel());
    }
}

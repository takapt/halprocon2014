//------------------------------------------------------------------------------
/// @file
/// @brief HPCRandomSet.hpp の実装
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2014 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCRandomSet.hpp"

#include "HPCCommon.hpp"

namespace hpc {
    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ///
    /// @param[in] aSeed 乱数のシードを表す構造体。
    RandomSet::RandomSet(const RandomSeed& aSeed)
        : mSystem(aSeed.x, aSeed.y)
        , mGame(aSeed.z, aSeed.w)
    {
    }

    //------------------------------------------------------------------------------
    /// @return システムで使用する乱数生成クラス
    Random& RandomSet::system()
    {
        return mSystem;
    }

    //------------------------------------------------------------------------------
    /// @return ゲーム中に使用する乱数生成クラス
    Random& RandomSet::game()
    {
        return mGame;
    }

    //------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
// EOF

/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app.cpp
 * @brief sf::app::* implementation for this application (11_app_controller):
 *        a custom IController (AppController), the stock estimator, and no
 *        additional task.
 *        本アプリ（11_app_controller）の sf::app::* 実装: 独自 IController
 *        （AppController）、標準推定器、追加タスクなし。
 *
 * @design app_hooks.hpp — sf::app::controller/estimator/start contract   [OK]
 * @design docs/plans/sf-app-sils-plan.md §4 Phase 2 — 11_app_controller   [OK]
 */

#include "app_hooks.hpp"
#include "app_controller.hpp"
#include "stock_hooks.hpp"

namespace sf::app {

sf::IController& controller()
{
    // Function-local static: constructed once (first call). init() is called
    // exactly once, on that first call, matching the app_hooks.hpp contract.
    // 関数内 static: 初回呼び出しで構築される。init() も最初の呼び出しで
    // 一度だけ実行する（app_hooks.hpp の契約どおり）。
    static AppController app_controller;
    static bool initialized = false;
    if (!initialized) {
        app_controller.init();
        initialized = true;
    }
    return app_controller;
}

sf::IEstimator& estimator()
{
    // This template only replaces the controller — the estimator stays the
    // vehicle's standard one (ESKF / complementary, by param estimator.type).
    // 本テンプレートはコントローラのみ差し替える — 推定器は vehicle 標準
    // （estimator.type で ESKF／相補）のまま。
    return stock::estimator();
}

void start()
{
    // No additional task in this template — see examples/12_app_task_hello
    // for an example that starts one.
    // 本テンプレートに追加タスクは無い — 追加タスクを起動する例は
    // examples/12_app_task_hello を参照。
}

}  // namespace sf::app

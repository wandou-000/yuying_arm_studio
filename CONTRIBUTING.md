# Contributing · 参与贡献

Thanks for your interest. Issues and pull requests are welcome.
感谢关注，欢迎提 Issue 和 PR。

## Before you start · 开始之前

- **Safety first.** Changes to `server.py` motion paths, `planner.js`, or `ik.js` can move real
  hardware. Test at low speed, with the workspace clear and the E-stop reachable.
  涉及运动下发的改动请低速试跑，确保工作空间无人、急停可及。
- Open an issue first for anything larger than a bug fix, so we can agree on the approach.
  较大改动请先开 Issue 讨论方案。

## Development · 开发

```bash
pip install -r requirements.txt
python server.py --assets /path/to/your_robot_model
```

There is no build step — edit files under `web/` and reload the browser.
前端无需构建，改完 `web/` 下的文件刷新浏览器即可。

You can work on most of the UI and planner without hardware: the backend runs fine with no serial
port connected, and `analyze_plan.py` works on exported plan dumps offline.
无硬件也能开发大部分 UI 与规划逻辑：后端不连串口也能启动，`analyze_plan.py` 可离线分析导出的
规划数据。

## Style · 代码风格

- Python: standard library only where practical; keep dependencies to `pyserial` + `numpy`.
  Python 侧尽量只用标准库，依赖控制在 `pyserial` + `numpy`。
- JavaScript: plain ES modules, no bundler, no new runtime dependencies. Third-party libraries go
  into `web/vendor/` with their license headers intact.
  JS 侧使用原生 ES 模块，不引入打包器和新的运行时依赖；第三方库放进 `web/vendor/` 并保留许可头。
- Comments and commit messages in English or Chinese are both fine.
  注释与提交信息中英文均可。

## Pull requests · 提交 PR

- Keep changes focused; one topic per PR. 一个 PR 只做一件事。
- Describe how you tested it, and whether it was tested on real hardware.
  说明你如何测试的，以及是否在真机上验证过。
- Do not commit URDF/STL model files, `plan_dumps/`, or `__pycache__/`.
  不要提交 URDF/STL 模型文件、`plan_dumps/` 和 `__pycache__/`。

## License · 许可

By contributing you agree that your contributions are licensed under the MIT License.
提交贡献即表示同意以 MIT 许可证授权。

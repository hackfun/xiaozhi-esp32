#ifndef RADAR_TOOLS_H
#define RADAR_TOOLS_H

namespace radar {

// 把雷达状态注册成 MCP 工具，让 LLM 能自己判断「还要走多远」。
// 在板型初始化时调用一次即可。
void RegisterRadarTools();

}  // namespace radar

#endif  // RADAR_TOOLS_H

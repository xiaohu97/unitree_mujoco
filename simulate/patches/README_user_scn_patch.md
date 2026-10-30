# vendored MuJoCo 补丁：让 user_scn 在托管模式下渲染

`simulate/mujoco/` 是外部依赖，**被 .gitignore 排除**（见仓库根 .gitignore），所以下面这两处改动
不会随 git 同步。在新机器上按 readme 放好 MuJoCo 源码后，需要手工把它们加回去。

## 为什么需要

`simulate/src/perception_overlay.h`（高程图 / 深度相机视野的叠加显示）靠 `mj::Simulate::user_scn`
把自定义几何体塞进渲染场景。但上游 simulate 只在 **passive 模式**（`is_passive=true`）才会把
`user_scn` 的几何体拷进场景，而本仓库的模拟器是**托管模式**（`main.cc` 里 `is_passive=false`），
不补这两处的话 `user_scn` 会被静默丢弃 —— 按 M / C 键没有任何反应，也没有报错。

## 补丁内容（都在 `simulate/mujoco/simulate/simulate.cc`）

### 1. `Simulate::Sync()`：托管分支也收集用户几何体

找到

```cpp
  if (!is_passive_) {
    mjv_updateScene(m_, d_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);
  } else {
```

把它改成

```cpp
  if (!is_passive_) {
    mjv_updateScene(m_, d_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);

    // [unitree_mujoco patch] 上游只在 passive 模式支持 user_scn。本仓库的模拟器是托管模式
    // (is_passive=false)，不补这一段的话 sim->user_scn 里的几何体会被静默丢弃。
    // 见 src/perception_overlay.h（高程图 / 深度相机视野叠加显示）。
    if (user_scn) {
      user_scn_geoms_.clear();
      user_scn_geoms_.reserve(user_scn->ngeom);
      for (int i = 0; i < user_scn->ngeom; ++i) {
        user_scn_geoms_.push_back(user_scn->geoms[i]);
      }
    }
  } else {
```

### 2. `Simulate::RenderLoop()`：托管分支把它们追加到场景

找到

```cpp
      // update scene, doing a full sync if in fully managed mode
      if (!is_passive_) {
        Sync();
      } else if (m_passive_ && d_passive_) {
```

把它改成

```cpp
      // update scene, doing a full sync if in fully managed mode
      if (!is_passive_) {
        Sync();

        // [unitree_mujoco patch] 配合 Sync() 里的同名补丁：把用户几何体追加到渲染场景
        if (user_scn) {
          int nusergeom = user_scn_geoms_.size();
          int ngeom = std::min(nusergeom, this->scn.maxgeom - this->scn.ngeom);
          if (ngeom < nusergeom && !this->scn.status) {
            mju_warning("Pre-allocated visual geom buffer is full. "
                        "Increase maxgeom above %d.", this->scn.maxgeom);
            this->scn.status = 1;
          }
          std::memcpy(this->scn.geoms + this->scn.ngeom, user_scn_geoms_.data(),
                      ngeom * sizeof(mjvGeom));
          this->scn.ngeom += ngeom;
        }
      } else if (m_passive_ && d_passive_) {
```

## 怎么确认打上了

```bash
grep -c "unitree_mujoco patch" simulate/mujoco/simulate/simulate.cc   # 应为 2
```

然后重新编译，起模拟器，点窗口按 **M**：平地上应出现一片绿色小球（策略消费的 17×11 高程图网格）。
没反应就是补丁没生效。

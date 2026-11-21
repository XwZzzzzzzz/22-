# eight 图像模块

`image.c` 和 `image.h` 是从 `C:\Users\1212\Desktop\eight` 原样复制的，SHA-256 与源文件一致。

## 接口

在单独使用这套源码时包含 `image_api.h`。主要调用顺序：

1. `Get_image(camera_image)` 复制摄像头灰度图。
2. `image_process()` 完成二值化和八邻域寻线。
3. 从 `bin_image` 读取二值图；`original_image` 保存灰度图。

其它算法入口见 `image_api.h`。

## 工程集成说明

当前 `D:\ai` 的 Keil 目标已经通过 `user\vision.c` 编译了同名图像处理函数。因此此目录作为原始源码和接口参考保留，没有加入 `mdk\seekfree.uvproj`，避免重复定义。

原始 `image.h` 自带整数类型定义，而本工程 C251 的 `uint32`/`int32` 使用 32 位 `long`。将原始 `.c` 直接加入当前 C251/C89 目标前，需要先统一类型定义并拆分重复实现；本次保留原始算法文件，没有改动其代码。

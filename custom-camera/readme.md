We have a custom MIPI camera board interfaced to the Jetson TX2i SOM on P3310 carrier board. The MIPI video output of the camera is similar to IMX219, operated in mode IMX219_3264x2464_21FPS. The custom camera board has no i2c line and the board starts streaming video as soon as it is powered on.

We are using tegra186-camera-e3326-a00.dtsi for device tree and OV5693 as a reference. for developing the driver.

We have gone through the sensor development guide and various posts on the forum and have followed following steps:

Disabled camera plugin manager and camera module and included e3326 dtsi as below
#include <t18x-common-platforms/tegra186-quill-common-p3489-1000-a00.dtsi>
#include <t18x-common-platforms/tegra186-quill-power-tree-p3489-1000-a00-00.dtsi>
/*#include <t18x-common-platforms/tegra186-quill-camera-modules.dtsi>*/
#include <t18x-common-modules/tegra186-display-e3320-1000-a00.dtsi>
#include <t18x-common-modules/tegra186-camera-e3326-a00.dtsi>
 
/* comms dtsi file should be included after gpio dtsi file */
#include <t18x-common-plugin-manager/tegra186-quill-p3489-1000-a00-plugin-manager.dtsi>
#include <t18x-common-modules/tegra186-super-module-e2614-p2597-1000-a00.dtsi>
#include <t18x-common-plugin-manager/tegra186-quill-display-plugin-manager.dtsi>
#include <t18x-common-prod/tegra186-priv-quill-p3489-1000-a00-prod.dtsi>
/*#include <t18x-common-plugin-manager/tegra186-quill-camera-plugin-manager.dtsi>*/



mclk_khz = "24000"; 
num_lanes = "2"; 
tegra_sinterface = "serial_a"; 
phy_mode = "DPHY"; 
discontinuous_clk = "no"; 
dpcm_enable = "false"; 
cil_settletime = "0"; 
active_w = "640"; 
active_h = "512"; 
pixel_t = "rgb_rgb88824"; 
readout_orientation = "90"; 
line_length = "2560"; 
inherent_gain = "1"; 
mclk_multiplier = "9.33"; 
pix_clk_hz = "198000000"; 
gain_factor = "16"; 
framerate_factor = "1000000"; 
exposure_factor = "1000000"; 
min_gain_val = "16"; /* 1.00x */ 
max_gain_val = "170"; /* 10.66x */ 
step_gain_val = "1"; 
default_gain = "16"; /* 1.00x */ 
min_hdr_ratio = "1"; 
max_hdr_ratio = "1"; 
min_framerate = "2000000"; /* 2.0 fps */ 
max_framerate = "60000000"; /* 60.0 fps */ 
step_framerate = "1"; 
default_framerate = "50000000"; /* 60.0 fps */ 
min_exp_time = "13"; /* us */ 
max_exp_time = "683709"; /* us */ 
step_exp_time = "1"; 
default_exp_time = "2495"; /* us */ 
embedded_metadata_height = "2";


частота clock mipi 198 
скорость линии 49,5 
частота кадров 50 
разрешение 800х600 (на данном образце) 
2 линии мипи 

и и2с вообще не задействован для инициализации видео поток гонится сразу при запуске камеры (если прям ничего не будет получаться, смогу проверить на малинке поток)
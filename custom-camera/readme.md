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
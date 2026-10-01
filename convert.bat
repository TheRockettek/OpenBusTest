cmake --build build-ode --config Release --target uno3d_converter

.\build-ode\Release\uno3d_converter.exe --workers 8 --convert-textures "C:\Program Files (x86)\Steam\steamapps\common\OMSI 2\Vehicles\MAN_DL05\Model\DL05.cfg" --out "MAN_DL05\Model\DL05_obj"
.\build-ode\Release\uno3d_converter.exe --workers 8 --convert-textures "C:\Program Files (x86)\Steam\steamapps\common\OMSI 2\Vehicles\[SP] Studio Polygon 400MMC\Model\Configuration Files\E400MMC_ADL_10.9m_Voith_LowHeight.cfg" --out "SP_E400MMC\Model\SP_E400MMC_obj"

.\build-ode\Release\obj_bundle.exe MAN_DL05\Model\DL05_obj MAN_DL05\Model\DL05_obj\openbus.obx
.\build-ode\Release\obj_bundle.exe SP_E400MMC\Model\SP_E400MMC_obj SP_E400MMC\Model\SP_E400MMC_obj\openbus.obx
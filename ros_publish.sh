#!/bin/bash

echo -n "Enter version number: "
read version
pkg_dir="R-Fans-ROS-Driver_v"$version
pkg_name="R-Fans-ROS-Driver_v"$version".zip"
echo "target file name: "$pkg_name
echo ""

mkdir -p $pkg_dir
cp -ravf ./release_note.txt $pkg_dir/

dir_tmp=$pkg_dir/ROSDriver/
mkdir -p $dir_tmp
cp -ravf ./CMakeLists.txt $dir_tmp
cp -ravf ./launch $dir_tmp
cp -ravf ./msg $dir_tmp
cp -ravf ./package.xml $dir_tmp
cp -ravf ./RFans_Rviz_cfg.rviz $dir_tmp
cp -ravf ./appolo_rviz_cfg.rviz $dir_tmp
cp -ravf ./src $dir_tmp
cp -ravf ./srv $dir_tmp
sync

zip -r $pkg_name $pkg_dir

if [ $? != 0 ]; then
    echo "excute cmd zip failed!"
    exit 1
fi

md5sum $pkg_name

echo ""
echo "publish ROS Driver success!"
echo ""
rm -rf $pkg_dir
sync







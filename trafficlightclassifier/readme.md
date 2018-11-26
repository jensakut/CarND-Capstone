
create an environment like this to be compatible with carla

conda create -n object-detection tensorflow-gpu==1.4.1 keras==2.0.8 scipy==0.19.1 numpy=1.13.1 Flask==0.11.1 matplotlib
pip install pillow
git clone https://github.com/tensorflow/models into trafficlightclassifier

download ssd_inception_v2_coco_2018_01_28 into the models subdir

for an older version which is compatible to the older tensorflow

git checkout f7e99c0 

sudo apt-get install protobuf-compiler python-pil python-lxml python-tk

conda activate object-detection





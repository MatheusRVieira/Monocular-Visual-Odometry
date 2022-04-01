
# Monocular visual Odometry C++

<p align="center">
  <img src="doc/monovo.png" height="600px"/>
</p>


This is a simple implementation that mostly uses OpenCV. However, I plan to replace major components
with different ideas, like using Deep Learning for features etc. Stay tuned and feel free to ask
me with any questions that you might have.


## Current algo is as follows

```
    1. Capture images: It, It+1,
    2. Undistort the above images. (using kitti so this step is done for you!)
    3. Use FAST algorithm to detect features in It, and track those features to It+1. A new detection is triggered if the number of features drop below a certain threshold.
    4. Use Nister’s 5-point alogirthm with RANSAC to compute the essential matrix.
    5. Estimate R,t from the essential matrix that was computed in the previous step.
    6. Take scale information from some external source (like a speedometer), and concatenate the translation vectors, and rotation matrices.

```



## Deps

1. OpenCV 3
2. cmake 3.1 or greater

## Dataset (Kitti Odometry: http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

1. Grab the grayscale
2. ground truth poses


## Building and Running
1. clone this repo
2. `mkdir build && cd build`
3. `cmake ..`
4. `make`
5. ./mono_vo

## FAST features Visualized

To see the FAST features visualized
1. `./mono_vo viz`

<p align="center">
  <img src="doc/FASTfeatures.png" width="750px" height="300px"/>
</p>


## TODOs

1. fix to work with any given sequence in the kitti dataset.
2. figure out a better way to estimate scale (perhaps use lidar!)
3. better document the code
4. DRY the code up.

## Credit
Major S/O to a blog http://avisingh599.github.io/vision/monocular-vo/ that gave the inspiration and a good starting point. Go read his blog to get a better understanding of the basics. :)


## Matheus Ribero (matheusrbv99@gmail.com)

## how to rename multiple files (a is the old prefix to be replaced, b is the new replacement)
$ mmv a\* b\#1

## Useful features using FFMPEG In UBUNTU

- Convert video to a image sequence

$ ffmpeg -i PERCURSO_FINAL.MOV -vf fps=10 %06d.png

- Convert video to a image sequence. Se quiser começar a sequencia a partir do 000179

$ ffmpeg -i 2.MOV -vf fps=10 -start_number 000179 %06d.png

- Convert video format

$ ffmpeg -i video.MOV -qscale 0 output.mp4

- change video resolution keeping quality

$ ffmpeg -i input.mp4 -vf scale=1280:720 -preset slow -crf 18 output.mp4

- Crop video (To crop a 80×60 section, starting from position (200, 100))

$ ffmpeg -i in.mp4 -filter:v "crop=80:60:200:100" -c:a copy out.mp4

- Split video into parts (0 eh o começo do video em segundos desejado. 30 eh a quantidade de segundos desejada)

$ ffmpeg -i IMG_1325.MOV  -ss 0 -t 30 RETORNO2.MOV

- Rotate 90º clockwise (2 = 90CounterClockwise)
$ ffmpeg -i in.MOV -vf "transpose=1" out.MOV

## Observações
- Tô utilizando uma sequência de imagens próprias a partir de um vídeo. Primeiramente, a conversão foi feita com 30 FPS, a trajetória da câmera ficou totalmente aleatória e sem nexo. Isso se deve ao fato de que com 30 FPS, os movimentos indesejaveis da câmera e consequentemente as features acabam representando movimentos aleatórios para os algortimos. Fiz a conversão com 10 FPS e funcionou excelente!

- Se durante o vídeo forem realizados movimentos bruscos no qual um segmente fique todo borrado, o seguinte erro aparecerá:

*Error terminate called after throwing an instance of 'cv::Exception' what():  OpenCV(4.5.5-dev) /home/matheus-ubuntu/opencv_build/opencv/modules/core/src/matrix.cpp:1175: error: (-13:Image step is wrong) The matrix is not continuous, thus its number of rows can not be changed in function 'reshape'
Trata-se da geração de uma matriz descontínua na função findessentialmatrix. Assim, a função recoverPose não consegue ler a matrix e gera o erro.*    

- A execução do algoritmo ficará lenta quanto mais for o tamanho da imagem em pixels (não necessariamente em megas) e quanto mais features forem detectadas em um segmento de imagens 


## Next steps: 
- [ ] undistorted images from calibration data
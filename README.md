PCL practice segmenting point clouds

Libraries that I use:
* PCL 1.8: http://pointclouds.org/
-- Installed on Mac 10.11 with Homebrew: http://www.pointclouds.org/documentation/tutorials/installing_homebrew.php

Data sets used:
* Oakland 3-D Point Cloud Dataset - CVPR 2009 subset
-- Link: http://www.cs.cmu.edu/~vmr/datasets/oakland_3d/cvpr09/doc/
-- From the paper:
--      Contextual Classification with Functional Max-Margin Markov Networks. 
--      Daniel Munoz, J. Andrew (Drew) Bagnell, Nicolas Vandapel, and Martial Hebert. 
--      IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR), June, 2009.

Making the files:
* `cmake -H. -Bbuild`
* `cmake --build build -- -j3`

Running the script:
* `./bin/point_cloud_seg`
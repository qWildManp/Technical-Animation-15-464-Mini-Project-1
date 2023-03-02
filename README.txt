Here is my implementations of miniproject 1 based on CMU ACM Viewer
I have implement two IK mechanics method: CCD IK and Transpose Jacobian IK, I tried using Pesudo-Jacobian IK,but seems invert matrix function have somthing wrong.

In the folder you can see:
1. Project Folder(“MiniProject-one”)
2. video illustraion about CCD IK
3. video illustraion about Transpose Jacobian IK
4. An powerpoint to illustrate my work

How to run the code:
1. VS 2017 IDE is needed
2. Copy the "MiniProject 1" folder to anywhere you want to place
3. Open it by VS 2017
4. Click "Play" button on the IDE


Basic control Scheme of the program:
Note: the direction is based on : The direction the skeleton is facing is the front
"Arrow Key Up" : move the yellow ball forward 1 Unit
"Arrow Key Down" : move the yellow ball backward Unit
"Arrow Key Left" : move the yellow ball left 1 Unit
"Arrow Key Right" : move the yellow ball right 1 Unit
"W" : move the yellow ball up 1 Unit
"S" : move the yellow ball down 1 Unit
"C" : let skeleton's right arm do CCD IK to reach the yellow ball
"J" : let skeleton's right arm do Jacobian IK to reach the yellow ball
"R" : stop doing all the IK stimulation


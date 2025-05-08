This repository is for the ball bearing sorting machine
The basis of this project was to train a model to be able to differentiate between 3 different kinds of balls.
The model uses mobilenet V2 features, then is trained as a tensorflow lite model to be used for raspberry pi applications. 
The training of the model is broken up into 2 parts. 
First, in the extract file, the script extracts the mobilenet features. 
Second, in the train file, the script actually trains the model, saving it in .h5 and .keras formats
The classify.py file utilizes the model. 
I have found a few things with this method.
The reason I split it up into two parts, is this process, escpecially when running it with a large enough data set, will overload the CPU, and will cause the pi to kill the script, or even shutdown. 
Thus, I broke it up into parts incase this does happen. 
The second thing is, when you are training it on very similar things, such as ball bearings. You have to have a very large dataset, as in 20k+ photos. 
This method works well for this application, when it was trained on pictures that were very similar to the environment that it would be tested in, it succeeded. 
When it was tested in a new environment, it failed, and would most likely look for things that were not actually the balls. 

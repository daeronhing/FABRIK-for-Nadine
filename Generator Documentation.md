# Generator

Current way of controlling Nadine’s joints is through a software called Wten.
This software can read and interpret the imported xml file and control all available joints of Nadine’s body.

Dictionary *test* contains 8-bit value of all joints. These values will be modified when any of the joint is moving.

Function *modify()* is used to modify the value in *test* .

Function *get()* is used to retrieve the value of certain keys in *test* . It accepts two types of parameter, **list** or a **single string**. 

Function *dict_values()* will return value of all keys in *test*

Function *convertor()* will return the 8-bit value of parameters.
***Constraints need to be modified for further use***.

Function *mjtg()* will return a trajectory as **list** for a joint using **minimum jerk algorithm**.
It is used in *create_trajectory()* to generate trajectory of multiple joints in a **list**.

Function *generate_line()* will return time stamp and 8-bit value of all joints in a **single string**.

All functions above are implemented in *generate()* and *plot()* to return readable file for Wten or graph for user to observe and analyze.

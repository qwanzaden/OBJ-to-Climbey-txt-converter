Run Obj_file_decoder.py using python 2.7 with no arguments to convert all valid .obj files into climbey custom level .txt files, which can be loaded by the VR game climbey.
Determines the type of block a object is in the climbey file (i.e. "Metal", "Jumpy", "Grabbable, ect...) by using the objects material. For example, if the material in the .obj file (labeled usemtl in the .obj file) is the string "Metal.002" then that object will be assigned the "Metal" climbey material (everything after the period is ignored).
Only rectangular objects are valid. Objects without exactly 8 vertexs are discarded, and non-rectangular 8-vertex objects will probably cause a error.
Three example levels and their original blender files are in the "Example Levels" folder
	Bare_minimum.obj - The minimum needed objects to make the level playable in climbey
	Base_custom_level.obj - A level with one of every kind of block in it
	Example_level - An example of a very simple level

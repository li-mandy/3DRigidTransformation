# 3DRigidTransformation
《Estimating 3-D Rigid Body Transformations: A Comparison of Four Major Algorithms》
It USES SVD Method to calculate T and t.

Just calculate the transformation matrix, 
You can figure it out A A point in the coordinate system P In the coordinate system B The coordinates of the corresponding points in,
namely R by 3x3 The transformation matrix of, 
t by 3x1 Displacement transformation vector, 
The coordinates of the points here are 3x1 The column vector 
Nonhomogeneous form, The homogeneous form is 4x1 Column vector, an extra element value is added 1 nothing more. 
In theory, given at least 3 Point to point, you can calculate R and t. 
Natural, More point to point , The more accurate the calculated transformation 

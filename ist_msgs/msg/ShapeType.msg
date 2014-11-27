#id of this particular description in the database,
# required to be able to find an object id which has this description by its id
int32 representation_id

#quadrics or ellipsoid representation type
int8 representation_type
int8 ANY_DESCRIPTION=0
int8 ELLIPSOID=1
int8 QUADRICS=2

 
#discrete
int8 value
int8 ANY_SHAPE_TYPE=0
int8 SHAPE_TYPE_1=1
int8 SHAPE_TYPE_2_A=2
int8 SHAPE_TYPE_2_B=3
int8 SHAPE_TYPE_3=4

#continuous
string[] parameters_type
float64[] parameters_data

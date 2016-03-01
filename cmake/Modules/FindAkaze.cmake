
IF(EXISTS "/usr/local/akaze/include/")
    SET(Akaze_INCLUDE_DIRS "/usr/local/akaze/include/")
elseif(EXISTS "/usr/local/include/akaze/")
    SET(Akaze_INCLUDE_DIRS "/usr/local/include/akaze/")
ENDIF(EXISTS "/usr/local/akaze/include/")


IF(EXISTS  "/usr/local/akaze/lib/libAKAZE.a")
    SET(Akaze_LIBRARIES  "/usr/local/akaze/lib/libAKAZE.a")
ELSEIF(EXISTS "/usr/local/lib/libAKAZE.a") 
    SET(Akaze_LIBRARIES  "/usr/local/lib/libAKAZE.a")
ENDIF(EXISTS "/usr/local/akaze/lib/libAKAZE.a")


message("{Akaze_INCLUDE_DIRS} = ${Akaze_INCLUDE_DIRS}")

TEMPLATE = app
CONFIG += console c++11 debug
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += /usr/local/include \
	/usr/local/opencv4/include/opencv4 \
	/usr/local/opencv4/include/opencv4/opencv2 \
        ../tensorflow \
	../tensorflow/bazel-genfiles \
	../tensorflow/tensorflow/contrib/makefile/gen/protobuf/include \
	../tensorflow/tensorflow/contrib/makefile/gen/host_obj \
	../tensorflow/tensorflow/contrib/makefile/gen/proto \
	../tensorflow/tensorflow/contrib/makefile/downloads/nsync/public  \
	../tensorflow/tensorflow/contrib/makefile/downloads/eigen  \
	../tensorflow/tensorflow/contrib/makefile/downloads/absl \
	../tensorflow/bazel-out/local_linux-py3-opt/genfiles  \
	../../opencv/include \
	/usr/local/include/eigen3 \
	../../opencv/opencv_contrib/modules/bgsegm/include \

LIBS += /usr/local/opencv4/lib/libopencv_calib3d.so \
        /usr/local/opencv4/lib/libopencv_core.so    \
        /usr/local/opencv4/lib/libopencv_highgui.so \
        /usr/local/opencv4/lib/libopencv_imgproc.so \
        /usr/local/opencv4/lib/libopencv_imgcodecs.so\
        /usr/local/opencv4/lib/libopencv_objdetect.so\
        /usr/local/opencv4/lib/libopencv_photo.so \
        /usr/local/opencv4/lib/libopencv_dnn.so \
        /usr/local/opencv4/lib/libopencv_features2d.so \
        /usr/local/opencv4/lib/libopencv_stitching.so \
        /usr/local/opencv4/lib/libopencv_flann.so\
        /usr/local/opencv4/lib/libopencv_videoio.so \
        /usr/local/opencv4/lib/libopencv_video.so\
        /usr/local/opencv4/lib/libopencv_ml.so \
    	../tensorflow/bazel-bin/tensorflow/libtensorflow_cc.so \
    	../tensorflow/bazel-bin/tensorflow/libtensorflow_framework.so \



SOURCES += \
    DeepAppearanceDescriptor/FeatureTensor.cpp \
    DeepAppearanceDescriptor/model.cpp \
    KalmanFilter/kalmanfilter.cpp \
    KalmanFilter/linear_assignment.cpp \
    KalmanFilter/nn_matching.cpp \
    KalmanFilter/track.cpp \
    KalmanFilter/tracker.cpp \
    MunkresAssignment/munkres/munkres.cpp \
    MunkresAssignment/hungarianoper.cpp \
    CountingBees.cpp \
    main.cpp \



HEADERS += \
    DeepAppearanceDescriptor/dataType.h \
    DeepAppearanceDescriptor/FeatureTensor.h \
    DeepAppearanceDescriptor/model.h \
    KalmanFilter/kalmanfilter.h \
    KalmanFilter/linear_assignment.h \
    KalmanFilter/nn_matching.h \
    KalmanFilter/track.h \
    KalmanFilter/tracker.h \
    MunkresAssignment/munkres/matrix.h \
    MunkresAssignment/munkres/munkres.h \
    MunkresAssignment/hungarianoper.h \
    CountingBees.h \

DEFINES += _GLIBCXX_USE_CXX11_ABI=0

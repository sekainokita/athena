QT.grpc.VERSION = 6.5.2
QT.grpc.name = QtGrpc
QT.grpc.module = Qt6Grpc
QT.grpc.libs = $$QT_MODULE_LIB_BASE
QT.grpc.ldflags = 
QT.grpc.includes = $$QT_MODULE_INCLUDE_BASE $$QT_MODULE_INCLUDE_BASE/QtGrpc
QT.grpc.frameworks = 
QT.grpc.bins = $$QT_MODULE_BIN_BASE
QT.grpc.depends =  core protobuf network
QT.grpc.uses = 
QT.grpc.module_config = v2
QT.grpc.DEFINES = QT_GRPC_LIB
QT.grpc.enabled_features = grpc native_grpc
QT.grpc.disabled_features = 
QT_CONFIG += grpc native_grpc
QT_MODULES += grpc


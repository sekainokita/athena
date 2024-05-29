/****************************************************************************
** Meta object code from reading C++ file 'logfilepositionsource.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../logfilepositionsource.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'logfilepositionsource.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS_t {};
constexpr auto qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS = QtMocHelpers::stringData(
    "LogFilePositionSource",
    "startUpdates",
    "",
    "stopUpdates",
    "updateGpsPosition",
    "getGpsHeading",
    "getGpsLatitude",
    "getGpsLongitude",
    "getGpsCvHeading",
    "getGpsCvLatitude",
    "getGpsCvLongitude",
    "requestUpdate",
    "timeout",
    "readNextPosition"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSLogFilePositionSourceENDCLASS[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   86,    2, 0x0a,    1 /* Public */,
       3,    0,   87,    2, 0x0a,    2 /* Public */,
       4,    0,   88,    2, 0x0a,    3 /* Public */,
       5,    0,   89,    2, 0x0a,    4 /* Public */,
       6,    0,   90,    2, 0x0a,    5 /* Public */,
       7,    0,   91,    2, 0x0a,    6 /* Public */,
       8,    0,   92,    2, 0x0a,    7 /* Public */,
       9,    0,   93,    2, 0x0a,    8 /* Public */,
      10,    0,   94,    2, 0x0a,    9 /* Public */,
      11,    1,   95,    2, 0x0a,   10 /* Public */,
      11,    0,   98,    2, 0x2a,   12 /* Public | MethodCloned */,
      13,    0,   99,    2, 0x08,   13 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::UInt,
    QMetaType::UInt,
    QMetaType::Double,
    QMetaType::Double,
    QMetaType::Double,
    QMetaType::Double,
    QMetaType::Double,
    QMetaType::Void, QMetaType::Int,   12,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject LogFilePositionSource::staticMetaObject = { {
    QMetaObject::SuperData::link<QGeoPositionInfoSource::staticMetaObject>(),
    qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSLogFilePositionSourceENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<LogFilePositionSource, std::true_type>,
        // method 'startUpdates'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'stopUpdates'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'updateGpsPosition'
        QtPrivate::TypeAndForceComplete<unsigned int, std::false_type>,
        // method 'getGpsHeading'
        QtPrivate::TypeAndForceComplete<unsigned int, std::false_type>,
        // method 'getGpsLatitude'
        QtPrivate::TypeAndForceComplete<double, std::false_type>,
        // method 'getGpsLongitude'
        QtPrivate::TypeAndForceComplete<double, std::false_type>,
        // method 'getGpsCvHeading'
        QtPrivate::TypeAndForceComplete<double, std::false_type>,
        // method 'getGpsCvLatitude'
        QtPrivate::TypeAndForceComplete<double, std::false_type>,
        // method 'getGpsCvLongitude'
        QtPrivate::TypeAndForceComplete<double, std::false_type>,
        // method 'requestUpdate'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'requestUpdate'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'readNextPosition'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void LogFilePositionSource::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<LogFilePositionSource *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->startUpdates(); break;
        case 1: _t->stopUpdates(); break;
        case 2: { uint _r = _t->updateGpsPosition();
            if (_a[0]) *reinterpret_cast< uint*>(_a[0]) = std::move(_r); }  break;
        case 3: { uint _r = _t->getGpsHeading();
            if (_a[0]) *reinterpret_cast< uint*>(_a[0]) = std::move(_r); }  break;
        case 4: { double _r = _t->getGpsLatitude();
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 5: { double _r = _t->getGpsLongitude();
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 6: { double _r = _t->getGpsCvHeading();
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 7: { double _r = _t->getGpsCvLatitude();
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 8: { double _r = _t->getGpsCvLongitude();
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 9: _t->requestUpdate((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 10: _t->requestUpdate(); break;
        case 11: _t->readNextPosition(); break;
        default: ;
        }
    }
}

const QMetaObject *LogFilePositionSource::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *LogFilePositionSource::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QGeoPositionInfoSource::qt_metacast(_clname);
}

int LogFilePositionSource::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGeoPositionInfoSource::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP

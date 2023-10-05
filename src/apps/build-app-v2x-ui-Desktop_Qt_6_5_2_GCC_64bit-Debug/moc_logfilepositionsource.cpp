/****************************************************************************
** Meta object code from reading C++ file 'logfilepositionsource.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.5.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ui/logfilepositionsource.h"
#include <QtCore/qmetatype.h>

#if __has_include(<QtCore/qtmochelpers.h>)
#include <QtCore/qtmochelpers.h>
#else
QT_BEGIN_MOC_NAMESPACE
#endif


#include <memory>

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'logfilepositionsource.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.5.2. It"
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
static constexpr auto qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS = QtMocHelpers::stringData(
    "LogFilePositionSource",
    "startUpdates",
    "",
    "stopUpdates",
    "requestUpdate",
    "timeout",
    "readNextPosition"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS_t {
    uint offsetsAndSizes[14];
    char stringdata0[22];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[12];
    char stringdata4[14];
    char stringdata5[8];
    char stringdata6[17];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS_t qt_meta_stringdata_CLASSLogFilePositionSourceENDCLASS = {
    {
        QT_MOC_LITERAL(0, 21),  // "LogFilePositionSource"
        QT_MOC_LITERAL(22, 12),  // "startUpdates"
        QT_MOC_LITERAL(35, 0),  // ""
        QT_MOC_LITERAL(36, 11),  // "stopUpdates"
        QT_MOC_LITERAL(48, 13),  // "requestUpdate"
        QT_MOC_LITERAL(62, 7),  // "timeout"
        QT_MOC_LITERAL(70, 16)   // "readNextPosition"
    },
    "LogFilePositionSource",
    "startUpdates",
    "",
    "stopUpdates",
    "requestUpdate",
    "timeout",
    "readNextPosition"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSLogFilePositionSourceENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   44,    2, 0x0a,    1 /* Public */,
       3,    0,   45,    2, 0x0a,    2 /* Public */,
       4,    1,   46,    2, 0x0a,    3 /* Public */,
       4,    0,   49,    2, 0x2a,    5 /* Public | MethodCloned */,
       6,    0,   50,    2, 0x08,    6 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    5,
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
        case 2: _t->requestUpdate((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 3: _t->requestUpdate(); break;
        case 4: _t->readNextPosition(); break;
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
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 5;
    }
    return _id;
}
QT_WARNING_POP

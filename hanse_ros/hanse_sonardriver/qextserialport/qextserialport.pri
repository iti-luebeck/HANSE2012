INCLUDEPATH += ../qextserialport

unix:DEFINES += OS_UNIX
win32:DEFINES += OS_WIN32

HEADERS                 += $$PWD/qextserialport.h \
                          $$PWD/qextserialenumerator.h \
                          $$PWD/qextserialport_global.h
SOURCES                 += $$PWD/qextserialport.cpp

unix:SOURCES           += $$PWD/posix_qextserialport.cpp
unix:!macx:SOURCES     += $$PWD/qextserialenumerator_unix.cpp
macx {
  SOURCES          += $$PWD/qextserialenumerator_osx.cpp
  LIBS             += -framework IOKit -framework CoreFoundation
}

win32 {
  SOURCES          += $$PWD/win_qextserialport.cpp $$PWD/qextserialenumerator_win.cpp
  DEFINES          += WINVER=0x0501 # needed for mingw to pull in appropriate dbt business...probably a better way to do this
  LIBS             += -lsetupapi
}

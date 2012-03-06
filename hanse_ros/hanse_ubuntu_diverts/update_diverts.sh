#!/bin/bash
	
# make sure we're running as root
if [ $UID -ne 0 ]; then
	exec sudo bash $0 $@
fi

MODE=update

if [ "$1" = remove ]; then
	MODE=remove
fi

function add_diversion {
	echo $MODE $1:
	echo -n "  "
	if [ $MODE = remove ]; then
		if [ "$(dpkg-divert --listpackage $1)" = LOCAL ]; then
			echo restoring distribution file
			rm -f $1
			dpkg-divert --quiet --rename --remove $1
		else
			echo not diverted
		fi
		return 1
	else
		if [ "$(dpkg-divert --listpackage $1)" = LOCAL ]; then
			echo updating diverted file
			rm -f $1
		else
			echo diverting file
			dpkg-divert --quiet --rename --add $1
		fi
        fi
}

function patch_diversion {
	if add_diversion $1; then
 		cp $1.distrib $1
		patch -s $1 $2
		rm -f $1.rej
	else
		return 1
	fi
}

function pyc_diversion {
	if add_diversion $1c; then # the c is not a typo
		python -m py_compile $1
	else
		return 1
	fi
}

MAKE_LIBRARY_PY=/opt/ros/electric/stacks/rosserial/rosserial_client/src/rosserial_client/make_library.py

patch_diversion $MAKE_LIBRARY_PY make_library.py.patch || true
pyc_diversion $MAKE_LIBRARY_PY || true
patch_diversion /usr/share/arduino/hardware/arduino/boards.txt boards.txt.patch || true


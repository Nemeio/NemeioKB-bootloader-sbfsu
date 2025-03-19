#!/bin/bash -
echo "Extract SE interface symbols"
BASEDIR=$(dirname "$0")

arm-none-eabi-nm $1 > ${BASEDIR}/nm.txt
case "$(uname -s)" in
    Linux*|Darwin*)
      tr -d '\015' <${BASEDIR}/se_interface.txt >${BASEDIR}/se_interface_unix.txt
      grep -F -f ${BASEDIR}/se_interface_unix.txt ${BASEDIR}/nm.txt > ${BASEDIR}/symbol.list
      rm ${BASEDIR}/se_interface_unix.txt
      ;;
    *)
      grep -F -f ${BASEDIR}/se_interface.txt ${BASEDIR}/nm.txt > ${BASEDIR}/symbol.list
      ;;
esac
wc -l ${BASEDIR}/symbol.list
cat ${BASEDIR}/symbol.list | awk '{split($0,a,/[ \r]/); print a[3]" = 0x"a[1]";"}' > $2
cat ${BASEDIR}/symbol.list
rm ${BASEDIR}/nm.txt
rm ${BASEDIR}/symbol.list

IP=${1-robot}
COLOR=${2-none}

if [[ ${IP} == robot ]]
then
    IP=10.1.22.91
elif [[ ${IP} == localhost ]]
then
    IP=127.0.0.1
elif [[ ${IP} == home ]]
then
    IP=192.168.1.242
fi

if [[ ${COLOR} == -h ]]
then
    echo "-c color"
    exit 1
fi

./client.exe ${IP} 4097 ${COLOR}

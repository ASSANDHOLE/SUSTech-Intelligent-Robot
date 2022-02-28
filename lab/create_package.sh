if [ $# -lt 2 ]; then
    echo $'Error: not enough arguments. \nUsage: create_workspace.sh <workspace_name> <package_name> [dependencies...]'
    exit 1
fi

dep=" "
for (( i=2; i <= "$#"; i++ )); do
    dep="$dep ${!i}"
done

cd ./"$1"
source ./devel/setup.bash
cd ./src
eval "catkin_create_pkg $dep"
cd ..
catkin_make

echo "Use: 'source ./$1/devel/setup.bash' to setup env" 

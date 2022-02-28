if [ -z "$1" ]; then
    echo $'Error: no workspace name given. \nUsage create_workspace.sh <workspace_name>'
    exit 1
fi

mkdir -p ./"$1"/src
cd ./"$1"/src
catkin_init_workspace
cd ..
catkin_make

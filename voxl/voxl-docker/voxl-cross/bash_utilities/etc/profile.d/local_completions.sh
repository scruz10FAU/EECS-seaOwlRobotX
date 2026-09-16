



__build_sh(){
    local DISTS=( "$( cat build.sh | grep AVAILABLE | cut -d '"' -f 2 )" )
    COMPREPLY=(" ")
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=( $(compgen -W '${DISTS}' -- ${COMP_WORDS[COMP_CWORD]}) )
        return 0
    fi
}

complete -F __build_sh ./build.sh



__deploy_sh(){
    echo -e "\nDeploy script should not be run from within a docker image"
    COMPREPLY=()
    return 0
}

complete -F __deploy_sh ./deploy_to_voxl.sh


__make_package_sh(){
    local OPTS=('deb timestamp')
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=( $(compgen -W '${OPTS}' -- ${COMP_WORDS[COMP_CWORD]}) )
        return 0
    fi
    return 0
}

complete -F __make_package_sh ./make_package.sh



__install_build_deps_sh(){
    DISTS=('qrb5165 qrb5165-2 qcs6490')
    AVAILABLE_SECTIONS=('dev staging')
    COMPREPLY=(" ")
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=( $(compgen -W '${DISTS}' -- ${COMP_WORDS[COMP_CWORD]}) )
        return 0
    elif [ "$COMP_CWORD" -eq 2 ]; then
        COMPREPLY=( $(compgen -W '${AVAILABLE_SECTIONS}' -- ${COMP_WORDS[COMP_CWORD]}) )
        return 0
    fi
}

complete -F __install_build_deps_sh ./install_build_deps.sh

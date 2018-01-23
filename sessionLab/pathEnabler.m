function pathEnabler(state)

switch state
    case 'activate'
        addpath('./..')
        addpath('./../networkShapes')
        addpath('./../controllers')
        addpath('./../sessionLab')
    case 'deactivate'
        rmpath('./..')
        rmpath('./../networkShapes')
        rmpath('./../controllers')
        rmpath('./../sessionLab')
end

end

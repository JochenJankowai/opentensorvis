# OpenTensorVis (Inviwo)

## Build instructions
For any of these papers, there is a shared set of steps that need to be taken in order to reproduce the results.
The very first step is to clone and configure the inviwo base repository on [Github](https://github.com/inviwo/inviwo/). This was last tested on commit `eeaa70bd8ed0047769bf8b723a3240d12df00dcf`, i.e.
```
git clone https://github.com/inviwo/inviwo.git .
git checkout eeaa70bd8ed0047769bf8b723a3240d12df00dcf
```

A step by step instruction manual can be found [here](https://inviwo.org/manual-gettingstarted-build.html).

Next, what you will need is the [modules](https://github.com/inviwo/modules) repository (@ `3795c7bd2ed7c85d5f669765771d0db2e498cc7e`) as well as the [OpenTensorVis](https://github.com/JochenJankowai/OpenTensorVis) repo. Clone these next to the inviwo repo and add the corresponding paths in CMake for the `IVW_EXTERNAL_MODULES` variable. For me, it looked like this: `C:\Users\jocja84\source\repos\OpenTensorVis\inviwo\opentensorvis;C:\Users\jocja84\source\repos\modules\misc`

The `misc` directory in the modules repo contains the VTK module and the OpenTensorVis repo contains the implementation for the papers.


### Trait-induced contour tree
In order to build 
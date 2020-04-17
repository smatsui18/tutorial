cmake_minimum_required( VERSION 2.8 )

### Create Project
project( testPCL )

set(CMAKE_PREFIX_NAME ${OPENCV_DIR} ${QTDIR})

### 必要？何やってるか調べる
set(CMAKE_CXX_STANDARD 14)

### mocで生成されたファイルをincludeするはずなので、有効にしとくが吉	※Qt用CMakeサンプルより
set(CMAKE_INCLUDE_CURRENT_DIR ON)
### mocコマンドを自動実行する	※Qt用CMakeサンプルより
set(CMAKE_AUTOMOC ON)

# Set StartUp Project (Option)	※PCL用CMakeサンプルより
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
# set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

### パッケージのパスを見つける
find_package(OpenCV 4.2.0 REQUIRED)
find_package(Qt5 REQUIRED Widgets)
find_package( PCL 1.10 REQUIRED )

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

### Sources
file(GLOB SOURCES "main.cpp")
#file(GLOB HEADERS "*.h")
#file(GLOB UI_SOURCES "*.ui")

### Include
#include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/include)
set(INCLUDE_DIR
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PCL_INCLUDE_DIRS}
  "C:/Program Files/VTK/"
#  C:/Qt/5.12.6/msvc2017_64/include
)
include_directories(INCLUDE_DIR)

### [C/C++]>[Preprocessor]>[Preprocessor Definitions]
add_definitions( ${PCL_DEFINITIONS} )


### .uiファイルをコンパイル可能なヘッダに変換（CMAKE_AUTOIC ONにて、add_executableに.ui直接渡せば良い説ある
#qt5_wrap_ui(UI_HEADERS ${UI_SOURCES})

### Linker
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

### 
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${UI_HEADERS})

### 追加ライブラリ
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
#target_link_libraries(${PROJECT_NAME} Qt5::3DCore)
#target_link_libraries(${PROJECT_NAME} Qt5::3DQuickExtras)
#target_link_libraries(${PROJECT_NAME} Qt5::3DInput)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} "C:/Program Files/VTK/lib")

### CMP0020 is not set と警告が出るため仮にOLDを指定。やって大丈夫？
#cmake_policy(SET CMP0020 NEW)
#cmake_policy(GET CMP0020 behavior)
#message("CMP0020: ${behavior}")
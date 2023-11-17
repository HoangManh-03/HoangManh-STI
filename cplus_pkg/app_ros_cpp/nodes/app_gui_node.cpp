/*
# MIT License

# Copyright (c) 2022 Kristopher Krasnosky

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
*/

#include "app_gui.h"
#include <thread>
#include <signal.h>

void signal_handler(int signal_num){
    AppGui self;
    self.shutdown_flag = 1;
    cout << "Program stop due to Ctrl C" << endl;
    QApplication::quit();
    ros::shutdown();
    exit(signal_num);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "app_gui_node",ros::init_options::AnonymousName);
  ros::NodeHandle n;
  signal(SIGABRT, signal_handler);
  QApplication a(argc, argv);	
  AppGui w;

	// load the icon from our qrc file and set it as the application icon

  try{
    thread th1(&AppGui::run, &w);
    // th1.join();
    w.setWindowTitle(QString::fromStdString(ros::this_node::getName()));

    // load the icon from our qrc file and set it as the application icon
    QIcon icon(":/icons/my_gui_icon.png");
    w.setWindowIcon(icon);

    w.show();
    try{
      a.exec();
    }
    catch(...){
      cout << "Cannot run app" << endl;
      w.shutdown_flag = 1;
      w.is_exist = 0;
    }
    th1.join();
  }
  catch(...){
    QApplication::quit();
    w.shutdown_flag = 1;
    w.is_exist = 0;
  }

  return 0;
}

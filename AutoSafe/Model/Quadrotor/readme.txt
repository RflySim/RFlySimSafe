改动：
1、使用了封装方法，将故障注入模块形成了一个总的库slx文件，通过修改库FaultModelLib的模块即能同步到模型。
2、使用了私有参数（提前定义好的，如故障ID等）和公共参数（主要是InParam参数，时Ints和Floats参数无法涉及到的参数）

3、此文件中的GenerateModelDLLFile.p支持将模型真值输出。

4、如果遇到初始化未变成Offboard模式，重新插拔飞控
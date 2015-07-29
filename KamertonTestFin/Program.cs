// Программа управления Камертон
// дата создания 10.06.2015 г.

using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Threading;
using System.Linq;


namespace KamertonTest
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {            
                string key = "gqwe71sqda1Fin";  //пишем тут любой ключ,это для проверки(вместо названия программы)
                using(Mutex mutex = new Mutex(false, key))
                {
                if(!mutex.WaitOne(0, false))
                {
                    MessageBox.Show("Программа уже открыта!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
                }
                else
                {
                 //   GC.Collect();
                    Application.EnableVisualStyles();
                    Application.SetCompatibleTextRenderingDefault(false);
                    Application.Run(new Form1());
                   // Application.Run(new PortChat());
                 }
              } 

        }
    }
}
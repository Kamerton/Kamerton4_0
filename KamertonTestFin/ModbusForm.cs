using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.IO.Ports;
using System.Globalization;
using System.Threading;
using KamertonTestFin;
using System.Linq;
using FieldTalk.Modbus.Master;
//using System.Runtime.Serialization.Formatters.Binary;


// Функции протокола MODBUS

//  res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);                  // 01 Считать бит  false или true по адресу 00000 - 09999
//  res = myProtocol.writeCoil(slave, startCoil, true);                               // 05   Записать бит false или true по адресу 00000 - 09999
//  res = myProtocol.forceMultipleCoils(slave, startCoil, coilVals, numCoils);        // 15 (0F) Записать бит false или true  по адресу 0-9999 
//  res = myProtocol.readInputDiscretes(slave, startCoil, coilArr, numCoils);         // 02  Считать бит  0 или 1 по адресу 10000 - 19999
//  res = myProtocol.readInputRegisters(slave, startRdReg, readVals, numRdRegs);      // 04  Считать бит  0 или 1 по адресу 30000 - 39999
//  res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);   // 03  Считать число из регистров по адресу  40000 -49999
//  res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs); // 16 (10Hex)Записать в регистры число по адресу 40000 -49999
//  res = myProtocol.writeSingleRegister(int slaveAddr,int regAddr, ushort regVal)    // 06  Записать в регистр число по адресу 40000 -49999


namespace KamertonTest
{

    public partial class Form1 : Form
    {

        private MbusMasterFunctions myProtocol;

        private int slave;
        private int startCoil;
        private int numCoils;
        private int startWrReg;
        private int numWrRegs;
        private int numRdRegs;
        private int startRdReg;
        private int res;
        private int TestN;
        private int TestRepeatCount;
        private int _SerialMonitor;
        static bool _All_Test_Stop = true;
        float temp_disp;

        public Form1()
        {
            InitializeComponent();
            LoadListboxes();
        }

        //SerialPort _serialPort = new SerialPort("COM4",
        //                                9600,
        //                                Parity.None,
        //                                8,
        //                                StopBits.One);

        const string fileName = "Kamerton log.txt";


        private void Form1_Load(object sender, EventArgs e)
        {
            ToolTip1.SetToolTip(txtPollDelay, "Задержка в миллисекундах между двумя последовательными операциями Modbus, 0 для отключения");
            ToolTip1.SetToolTip(cmbRetry, "Сколько раз повторить операцию, если в первый раз не принят?");
            ToolTip1.SetToolTip(cmbSerialProtocol, "Выбор протокола COM: ASCII или RTU");
            ToolTip1.SetToolTip(cmbTcpProtocol, "Выбор протокола Ethernet: MODBUS/TCP или Encapsulated RTU над TCP");
            //   ToolTip1.SetToolTip(textBox1, "Время задержки измерения в миллисекундах");
            cmbComPort.SelectedIndex = 0;
            cmbParity.SelectedIndex = 0;
            cmbStopBits.SelectedIndex = 0;
            cmbDataBits.SelectedIndex = 0;
            cmbBaudRate.SelectedIndex = 5;
            cmbSerialProtocol.SelectedIndex = 0;
            cmbTcpProtocol.SelectedIndex = 0;
            cmbRetry.SelectedIndex = 2;
            serial_connect();
            cmbCommand.SelectedIndex = 0;
            Polltimer1.Enabled = true;
            timer_byte_set.Enabled = false;
            timer_Mic_test.Enabled = false;
            timerCTS.Enabled = false;
            timerTestAll.Enabled = false;
            radioButton1.Checked = true;
            serviceSet();
            _SerialMonitor = 0;
            // _serialPort.Handshake = Handshake.None;
            // _serialPort.DataReceived += new SerialDataReceivedEventHandler(sp_DataReceived);
            // _serialPort.ReadTimeout  = 500;
            // _serialPort.WriteTimeout = 500;
            // if (!(_serialPort.IsOpen))
            // _serialPort.Open();
        }

        private delegate void SetTextDeleg(string text);                  //             

        void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            Thread.Sleep(500);
            //  string data = _serialPort.ReadLine();
            //  Привлечение делегата на потоке UI, и отправка данных, которые
            //  были приняты привлеченным методом.
            //  ---- Метод "si_DataReceived" будет выполнен в потоке UI,
            //  который позволит заполнить текстовое поле TextBox.
            //  this.BeginInvoke(new SetTextDeleg(si_DataReceived), new object[] { data });
        }

        private void serviceSet()
        {

            checkBoxSenAll.Checked = true;
        }

        private void si_DataReceived(string data)
        {
            switch (_SerialMonitor) // Переключатель экранов вывода информации с Serial
            {
                default:
                case 0:
                    //textBox9.Text += (data.Trim() + "\r\n");
                    //textBox9.Refresh();
                    textBox11.Text += (data.Trim() + "\r\n");
                    textBox11.Refresh();
                    break;
                case 1:
                    //textBox9.Text += (data.Trim() + "\r\n");
                    //textBox9.Refresh();
                    break;
                case 2:
                    break;
                case 3:
                    textBox11.Text += (data.Trim() + "\r\n");
                    textBox11.Refresh();
                    break;
            }
        }

        private void cmdOpenSerial_Click(object sender, EventArgs e)
        {

            //
            // First we must instantiate class if we haven't done so already
            //
            if ((myProtocol == null))
            {
                try
                {
                    if ((cmbSerialProtocol.SelectedIndex == 0))
                        myProtocol = new MbusRtuMasterProtocol(); // RTU
                    else
                        myProtocol = new MbusAsciiMasterProtocol(); // ASCII
                }
                catch (OutOfMemoryException ex)
                {
                    lblResult.Text = (" Ошибка была" + ex.Message);
                    label78.Text = ("Не удалось создать экземпляр класса серийного протокола!" + ex.Message);
                    return;
                }
            }
            else // already instantiated, close protocol, reinstantiate
            {
                if (myProtocol.isOpen())
                    myProtocol.closeProtocol();
                myProtocol = null;
                try
                {
                    if ((cmbSerialProtocol.SelectedIndex == 0))
                        myProtocol = new MbusRtuMasterProtocol(); // RTU
                    else
                        myProtocol = new MbusAsciiMasterProtocol(); // ASCII
                }
                catch (OutOfMemoryException ex)
                {
                    lblResult.Text = (" Ошибка была" + ex.Message);
                    label78.Text = ("Не удалось создать экземпляр класса серийного протокола!" + ex.Message);
                    return;
                }
            }
            //
            // Here we configure the protocol
            // Здесь мы настроим протокол
            int retryCnt;
            int pollDelay;
            int timeOut;
            int baudRate;
            int parity;
            int dataBits;
            int stopBits;
            int res;
            try
            {
                retryCnt = int.Parse(cmbRetry.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                retryCnt = 2;
            }
            try
            {
                pollDelay = int.Parse(txtPollDelay.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                pollDelay = 10;
            }
            try
            {
                timeOut = int.Parse(txtTimeout.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                timeOut = 1000;
            }
            try
            {
                baudRate = int.Parse(cmbBaudRate.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                baudRate = 57600;
            }
            switch (cmbParity.SelectedIndex)
            {
                default:
                case 0:
                    parity = MbusSerialMasterProtocol.SER_PARITY_NONE;
                    break;
                case 1:
                    parity = MbusSerialMasterProtocol.SER_PARITY_EVEN;
                    break;
                case 2:
                    parity = MbusSerialMasterProtocol.SER_PARITY_ODD;
                    break;
            }
            switch (cmbDataBits.SelectedIndex)
            {
                default:
                case 0:
                    dataBits = MbusSerialMasterProtocol.SER_DATABITS_8;
                    break;
                case 1:
                    dataBits = MbusSerialMasterProtocol.SER_DATABITS_7;
                    break;
            }
            switch (cmbStopBits.SelectedIndex)
            {
                default:
                case 0:
                    stopBits = MbusSerialMasterProtocol.SER_STOPBITS_1;
                    break;
                case 1:
                    stopBits = MbusSerialMasterProtocol.SER_STOPBITS_2;
                    break;
            }
            myProtocol.timeout = timeOut;
            myProtocol.retryCnt = retryCnt;
            myProtocol.pollDelay = pollDelay;
            // Note: The following cast is required as the myProtocol object is declared
            // as the superclass of MbusSerialMasterProtocol. That way myProtocol can
            // represent different protocol types.
            // Примечание: В следующем варианте требуется как объект myProtocol объявлен
            // Как суперкласс MbusSerialMasterProtocol. Таким образом myProtocol может
            // Представляют различные типы протоколов.

            res = ((MbusSerialMasterProtocol)(myProtocol)).openProtocol(cmbComPort.Text, baudRate, dataBits, stopBits, parity);
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                label78.Text = ("Последовательный порт успешно открыт с параметрами:  "
                            + (cmbComPort.Text + (", "
                            + (baudRate + (" baud, "
                            + (dataBits + (" data bits, "
                            + (stopBits + (" stop bits, parity " + parity)))))))));
                button5.Enabled = true;
                Polltimer1.Enabled = true;
            }
            else
            {
                lblResult.Text = (" ошибка была: " + BusProtocolErrors.getBusProtocolErrorText(res));
                label78.Text = ("Не удалось открыть протокол!");
            }
        }

        private void serial_connect()
        {
            //
            // First we must instantiate class if we haven't done so already
            //
            //  CloseButton.Enabled = true;
            //  cmdOpenSerial.Enabled = false;


            if ((myProtocol == null))
            {
                try
                {
                    if ((cmbSerialProtocol.SelectedIndex == 0))
                        myProtocol = new MbusRtuMasterProtocol(); // RTU
                    else
                        myProtocol = new MbusAsciiMasterProtocol(); // ASCII
                }
                catch (OutOfMemoryException ex)
                {
                    toolStripStatusLabel1.Text = ("Не удалось создать экземпляр класса серийного протокола! Ошибка была " + ex.Message);
                    return;
                }
            }
            else // already instantiated, close protocol, reinstantiate
            {
                if (myProtocol.isOpen())
                    myProtocol.closeProtocol();
                myProtocol = null;
                try
                {
                    if ((cmbSerialProtocol.SelectedIndex == 0))
                        myProtocol = new MbusRtuMasterProtocol(); // RTU
                    else
                        myProtocol = new MbusAsciiMasterProtocol(); // ASCII
                }
                catch (OutOfMemoryException ex)
                {
                    toolStripStatusLabel1.Text = ("Не удалось создать экземпляр класса серийного протокола! Ошибка была " + ex.Message);
                    return;
                }
            }
            //
            // Here we configure the protocol
            //
            short[] readVals = new short[125];
            int retryCnt;
            int pollDelay;
            int timeOut;
            int baudRate;
            int parity;
            int dataBits;
            int stopBits;
            int res;
            int slave;
            int startRdReg;
            int numRdRegs;
            try
            {
                retryCnt = int.Parse(cmbRetry.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                retryCnt = 0;
            }
            try
            {
                pollDelay = int.Parse(txtPollDelay.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                pollDelay = 0;
            }
            try
            {
                timeOut = int.Parse(txtTimeout.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                timeOut = 1000;
            }
            try
            {
                baudRate = int.Parse(cmbBaudRate.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                baudRate = 57600;
            }
            switch (cmbParity.SelectedIndex)
            {
                default:
                case 0:
                    parity = MbusSerialMasterProtocol.SER_PARITY_NONE;
                    break;
                case 1:
                    parity = MbusSerialMasterProtocol.SER_PARITY_EVEN;
                    break;
                case 2:
                    parity = MbusSerialMasterProtocol.SER_PARITY_ODD;
                    break;
            }
            switch (cmbDataBits.SelectedIndex)
            {
                default:
                case 0:
                    dataBits = MbusSerialMasterProtocol.SER_DATABITS_8;
                    break;
                case 1:
                    dataBits = MbusSerialMasterProtocol.SER_DATABITS_7;
                    break;
            }
            switch (cmbStopBits.SelectedIndex)
            {
                default:
                case 0:
                    stopBits = MbusSerialMasterProtocol.SER_STOPBITS_1;
                    break;
                case 1:
                    stopBits = MbusSerialMasterProtocol.SER_STOPBITS_2;
                    break;
            }
            myProtocol.timeout = timeOut;
            myProtocol.retryCnt = retryCnt;
            myProtocol.pollDelay = pollDelay;
            // Note: The following cast is required as the myProtocol object is declared 
            // as the superclass of MbusSerialMasterProtocol. That way myProtocol can
            // represent different protocol types.
            res = ((MbusSerialMasterProtocol)(myProtocol)).openProtocol(cmbComPort.Text, baudRate, dataBits, stopBits, parity);
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = ("Последовательный порт успешно открыт с параметрами: "
                    //lblResult.Text = ("Последовательный порт успешно открыт с параметрами: "
                            + (cmbComPort.Text + (", "
                            + (baudRate + (" baud, "
                            + (dataBits + (" data bits, "
                            + (stopBits + (" stop bits, parity " + parity)))))))));
            }
            else
            {
                toolStripStatusLabel1.Text = ("Не удалось открыть протокол, ошибка была: " + BusProtocolErrors.getBusProtocolErrorText(res));
            }

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            startRdReg = 47; // 40047 Адрес дата/время контроллера
            numRdRegs = 8;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
            lblResult.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {


                label83.Text = "";
                label83.Text = (label83.Text + readVals[0] + "." + readVals[1] + "." + readVals[2] + "   " + readVals[3] + ":" + readVals[4] + ":" + readVals[5]);


            }
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {

                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

            }
            else
            {
                toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                toolStripStatusLabel1.BackColor = Color.Red;
                Polltimer1.Enabled = false;
                timer_Mic_test.Enabled = false;
            }


        }

        private void file_fakt_namber()
        {
            short[] readVals = new short[125];
            int slave;
            int startRdReg;
            int numRdRegs;
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
            startRdReg = 112; // 40046 Адрес дата/время контроллера  
            numRdRegs = 4;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
            lblResult.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

                // textBox9.Text = "\r\n";
                textBox9.Text += "Текущий номер файла   -   ";
                textBox9.Text += (readVals[0]);
                if (readVals[1] < 10)
                {
                    textBox9.Text += ("0" + readVals[1]);
                }
                else
                {
                    textBox9.Text += (readVals[1]);
                }
                if (readVals[2] < 10)
                {
                    textBox9.Text += ("0" + readVals[2]);
                }
                else
                {
                    textBox9.Text += (readVals[2]);
                }
                if (readVals[3] < 10)
                {
                    textBox9.Text += ("0" + readVals[3] + ".TXT" + "\r\n");
                }
                else
                {
                    textBox9.Text += (readVals[3] + ".TXT" + "\r\n");
                }
            }

            else
            {
                toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                toolStripStatusLabel1.BackColor = Color.Red;
            }
        }

        #region Load Listboxes
        private void LoadListboxes()
        {
            //Three to load - ports, baudrates, datetype.  Also set default textbox values:
            //1) Available Ports:
            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                cmbComPort.Items.Add(port);
            }

            cmbComPort.SelectedIndex = 0;

        }
        #endregion

        #region timer all
        private void Polltimer1_Tick(object sender, EventArgs e)           // Выполняет контроль MODBUS и часов
        {
            short[] readVals = new short[125];
            int slave;
            int startRdReg;
            int numRdRegs;
            int res;

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            startRdReg = 46; // 40046 Адрес дата/время контроллера  
            numRdRegs = 8;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
            lblResult.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

                label83.Text = "";
                label83.Text = (label83.Text + readVals[0] + "." + readVals[1] + "." + readVals[2] + "   " + readVals[3] + ":" + readVals[4] + ":" + readVals[5]);


                startRdReg = 112; // 40046 Адрес дата/время контроллера  
                numRdRegs = 4;
                res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
                lblResult.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                if ((res == BusProtocolErrors.FTALK_SUCCESS))
                {
                    toolStripStatusLabel1.Text = "    MODBUS ON    ";
                    toolStripStatusLabel1.BackColor = Color.Lime;

                    label134.Text = "";
                    label134.Text = (label134.Text + readVals[0]);
                    if (readVals[1] < 10)
                    {
                        label134.Text += ("0" + readVals[1]);
                    }
                    else
                    {
                        label134.Text += (readVals[1]);
                    }
                    if (readVals[2] < 10)
                    {
                        label134.Text += ("0" + readVals[2]);
                    }
                    else
                    {
                        label134.Text += (readVals[2]);
                    }
                    if (readVals[3] < 10)
                    {
                        label134.Text += ("0" + readVals[3] + ".TXT");
                    }
                    else
                    {
                        label134.Text += (readVals[3] + ".TXT");
                    }

                }
            }

            else
            {
                toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                toolStripStatusLabel1.BackColor = Color.Red;
                timer_byte_set.Enabled = false;
                timer_Mic_test.Enabled = false;
                timerCTS.Enabled = false;
                timerTestAll.Enabled = false;
            }

            label80.Text = DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture);
            toolStripStatusLabel2.Text = ("Время : " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture));
            //  timer_Mic_test.Enabled = false;
        }

        private void timer_byte_set_Tick(object sender, EventArgs e)
        {
            short[] readVals = new short[124];
            int slave;
            int startRdReg;
            int numRdRegs;
            int startCoil;
            int numCoils;
            int i;
            int res;
            bool[] coilVals = new bool[64];
            bool[] coilArr = new bool[64];
            bool[] coilSensor = new bool[64];

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            //*************************  Получить данные состояния модуля Камертон ************************************

            Int64 binaryHolder;
            string binaryResult = "";
            int decimalNum;
            bool[] Dec_bin = new bool[64];
            startRdReg = 1;
            numRdRegs = 7;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);    // 03  Считать число из регистров по адресу  40000 -49999
            label78.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

                for (int bite_x = 0; bite_x < 7; bite_x++)
                {
                    // textBox11.Text += (bite_x + "  ");
                    decimalNum = readVals[bite_x];
                    //    textBox11.Text += (decimalNum +  "\r\n");
                    while (decimalNum > 0)
                    {
                        binaryHolder = decimalNum % 2;
                        binaryResult += binaryHolder;
                        decimalNum = decimalNum / 2;
                    }

                    int len_str = binaryResult.Length;

                    while (len_str < 8)
                    {
                        binaryResult += 0;
                        len_str++;
                    }

                    //****************** Перемена битов ***************************
                    //binaryArray = binaryResult.ToCharArray();
                    //Array.Reverse(binaryArray);
                    //binaryResult = new string(binaryArray);
                    //*************************************************************

                    //  textBox11.Text = (textBox11.Text + (binaryResult + "\r\n"));

                    for (i = 0; i < 8; i++)                         // 
                    {
                        if (binaryResult[i] == '1')
                        {
                            Dec_bin[i + (8 * bite_x)] = true;
                        }
                        else
                        {
                            Dec_bin[i + (8 * bite_x)] = false;
                        }
                        //   textBox11.Text = (textBox11.Text + (Dec_bin[i+(8*bite_x)] + " "));
                    }
                    binaryResult = "";
                    //textBox11.Text += ("\r\n");
                }

            }

            //*************************** Вывод состояния битов Камертона *****************************************

            label30.Text = "";
            label31.Text = "";
            label32.Text = "";

            label33.Text = "";
            label34.Text = "";
            label35.Text = "";
            label36.Text = "";

            for (i = 7; i >= 0; i--)
            {
                if (Dec_bin[i] == true)
                {
                    label30.Text += ("1" + "  ");
                }
                else
                {
                    label30.Text += ("0" + "  ");
                }
                if (Dec_bin[i + 8] == true)
                {
                    label31.Text += ("1" + "  ");
                }
                else
                {
                    label31.Text += ("0" + "  ");
                }


                if (Dec_bin[i + 16] == true)
                {
                    label32.Text += ("1" + "  ");
                }
                else
                {
                    label32.Text += ("0" + "  ");
                }

                if (Dec_bin[i + 24] == true)
                {
                    label33.Text += ("1" + "  ");
                }
                else
                {
                    label33.Text += ("0" + "  ");
                }

                if (Dec_bin[i + 32] == true)
                {
                    label34.Text += ("1" + "  ");
                }
                else
                {
                    label34.Text += ("0" + "  ");
                }

                if (Dec_bin[i + 40] == true)
                {
                    label35.Text += ("1" + "  ");
                }
                else
                {
                    label35.Text += ("0" + "  ");
                }
                if (Dec_bin[i + 48] == true)
                {
                    label36.Text += ("1" + "  ");
                }
                else
                {
                    label36.Text += ("0" + "  ");
                }
            }

            //***********************************************************************************

            if (Dec_bin[24] == false) // 30024 флаг подключения ГГ Радио2
            {
                label103.BackColor = Color.Red;
                label103.Text = "0";
            }
            else
            {
                label103.BackColor = Color.Lime;
                label103.Text = "1";
            }
            if (Dec_bin[25] == false) // 30025 флаг подключения ГГ Радио1
            {
                label104.BackColor = Color.Red;
                label104.Text = "0";
            }
            else
            {
                label104.BackColor = Color.Lime;
                label104.Text = "1";
            }

            if (Dec_bin[26] == false) // 30026 флаг подключения трубки
            {
                label105.BackColor = Color.Red;
                label105.Text = "0";
            }
            else
            {
                label105.BackColor = Color.Lime;
                label105.Text = "1";
            }

            if (Dec_bin[27] == false)   // 30027 флаг подключения ручной тангенты
            {
                label106.BackColor = Color.Red;
                label106.Text = "0";
            }
            else
            {
                label106.BackColor = Color.Lime;
                label106.Text = "1";
            }

            if (Dec_bin[28] == false)  // 30028 флаг подключения педали
            {
                label107.BackColor = Color.Red;
                label107.Text = "0";
            }
            else
            {
                label107.BackColor = Color.Lime;
                label107.Text = "1";
            }

            if (Dec_bin[40] == false) // 30040  флаг подключения магнитофона
            {
                label108.BackColor = Color.Red;
                label108.Text = "0";
            }
            else
            {
                label108.BackColor = Color.Lime;
                label108.Text = "1";
            }

            if (Dec_bin[41] == false) // 30041  флаг подключения гарнитуры инструктора 2 наушниками
            {
                label109.BackColor = Color.Red;
                label109.Text = "0";
            }
            else
            {
                label109.BackColor = Color.Lime;
                label109.Text = "1";
            }

            if (Dec_bin[42] == false) // 30042  флаг подключения гарнитуры инструктора
            {
                label110.BackColor = Color.Red;
                label110.Text = "0";
            }
            else
            {
                label110.BackColor = Color.Lime;
                label110.Text = "1";
            }

            if (Dec_bin[43] == false) // 30043  флаг подключения гарнитуры диспетчера с 2 наушниками
            {
                label111.BackColor = Color.Red;
                label111.Text = "0";
            }
            else
            {
                label111.BackColor = Color.Lime;
                label111.Text = "1";
            }

            if (Dec_bin[44] == false) // 30044  флаг подключения гарнитуры диспетчера
            {
                label112.BackColor = Color.Red;
                label112.Text = "0";
            }
            else
            {
                label112.BackColor = Color.Lime;
                label112.Text = "1";
            }

            if (Dec_bin[45] == false) // 30045  флаг подключения микрофона XS1 - 6 Sence
            {
                label113.BackColor = Color.Red;
                label113.Text = "0";
            }
            else
            {
                label113.BackColor = Color.Lime;
                label113.Text = "1";
            }

            if (Dec_bin[46] == false) //  30046  флаг подключения ГГС
            {
                label115.BackColor = Color.Red;
                label115.Text = "0";
            }
            else
            {
                label115.BackColor = Color.Lime;
                label115.Text = "1";
            }


            if (Dec_bin[52] == false) // 30052   флаг выключения ГГС (Mute)
            {
                label144.BackColor = Color.Red;
                label144.Text = "0";
            }
            else
            {
                label144.BackColor = Color.Lime;
                label144.Text = "1";
            }

            if (Dec_bin[53] == false) // 30053   флаг радиопередачи
            {
                label143.BackColor = Color.Red;
                label143.Text = "0";
            }
            else
            {
                label143.BackColor = Color.Lime;
                label143.Text = "1";
            }

            if (Dec_bin[54] == false) // 30054   флаг управления микрофонами гарнитур
            {
                label142.BackColor = Color.Red;
                label142.Text = "0";
            }
            else
            {
                label142.BackColor = Color.Lime;
                label142.Text = "1";
            }

            //********************Вторая колонка ********************
            startCoil = 1;  //  regBank.add(00001-12);   Отображение соостояния реле 0-9
            numCoils = 8;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));


            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {

                if (coilArr[0] == true)                              //   Реле RL0
                {
                    button37.BackColor = Color.Lime;
                    button48.BackColor = Color.White;
                }
                else
                {
                    button48.BackColor = Color.Red;
                    button37.BackColor = Color.White;
                }
                if (coilArr[1] == true)                              //   Реле RL1
                {
                    button40.BackColor = Color.Lime;
                    button53.BackColor = Color.White;
                }
                else
                {
                    button53.BackColor = Color.Red;
                    button40.BackColor = Color.White;
                }
                if (coilArr[2] == true)                              //   Реле RL2
                {
                    button44.BackColor = Color.Lime;
                    button79.BackColor = Color.White;
                }
                else
                {
                    button79.BackColor = Color.Red;
                    button44.BackColor = Color.White;
                }
                if (coilArr[3] == true)                              //   Реле RL3
                {
                    button49.BackColor = Color.Lime;
                    button66.BackColor = Color.White;
                }
                else
                {
                    button66.BackColor = Color.Red;
                    button49.BackColor = Color.White;
                }
                if (coilArr[4] == true)                              //   Реле RL4
                {
                    button38.BackColor = Color.Lime;
                    button52.BackColor = Color.White;
                }
                else
                {
                    button52.BackColor = Color.Red;
                    button38.BackColor = Color.White;
                }
                if (coilArr[5] == true)                              //   Реле RL5
                {
                    button71.BackColor = Color.Lime;
                    button47.BackColor = Color.White;
                }
                else
                {
                    button47.BackColor = Color.Red;
                    button71.BackColor = Color.White;
                }
                if (coilArr[6] == true)                              //   Реле RL6
                {
                    button69.BackColor = Color.Lime;
                    button42.BackColor = Color.White;
                }
                else
                {
                    button42.BackColor = Color.Red;
                    button69.BackColor = Color.White;
                }
                if (coilArr[7] == true)                              //   Реле RL7
                {
                    button51.BackColor = Color.Lime;
                    button45.BackColor = Color.White;
                }
                else
                {
                    button45.BackColor = Color.Red;
                    button51.BackColor = Color.White;
                }

            }

            startCoil = 9;  //  regBank.add(00009-16);   Отображение соостояния реле 9-16
            numCoils = 8;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {

                if (coilArr[0] == true)                              //   Реле RL8 Звук на микрофон regBank.add(9)
                {
                    button46.BackColor = Color.Lime;
                    button50.BackColor = Color.White;
                }
                else
                {
                    button50.BackColor = Color.Red;
                    button46.BackColor = Color.White;
                }
                if (coilArr[1] == true)                              //   Реле RL9  XP1 10 regBank.add(10)
                {
                    button27.BackColor = Color.Lime;
                    button30.BackColor = Color.White;
                }
                else
                {
                    button30.BackColor = Color.Red;
                    button27.BackColor = Color.White;
                }

                if (coilArr[2] == true)                              //   Свободен regBank.add(11)
                {
                    //button27.BackColor = Color.Lime;
                    //button30.BackColor = Color.White;
                }
                else
                {
                    //button30.BackColor = Color.Red;
                    //button27.BackColor = Color.White;
                }

                if (coilArr[3] == true)                                //   Свободен regBank.add(12)
                {
                    //button27.BackColor = Color.Lime;
                    //button30.BackColor = Color.White;
                }
                else
                {
                    //button30.BackColor = Color.Red;
                    //button27.BackColor = Color.White;
                }


                if (coilArr[4] == true)                             // XP8 - 2  Sence Танг н. regBank.add(13)
                {
                    button59.BackColor = Color.Lime;
                    button74.BackColor = Color.White;
                }
                else
                {
                    button74.BackColor = Color.Red;
                    button59.BackColor = Color.White;
                }
                if (coilArr[5] == true)                             //XP8 - 1  PTT Танг н. regBank.add(14)
                {
                    button39.BackColor = Color.Lime;
                    button41.BackColor = Color.White;
                }
                else
                {
                    button41.BackColor = Color.Red;
                    button39.BackColor = Color.White;
                }
                if (coilArr[6] == true)                             // XS1 - 5   PTT Мик  regBank.add(15)
                {
                    button7.BackColor = Color.Lime;
                    button18.BackColor = Color.White;
                }
                else
                {
                    button18.BackColor = Color.Red;
                    button7.BackColor = Color.White;
                }
                if (coilArr[7] == true)                             // XS1 - 6 Sence Мик. regBank.add(16)
                {
                    button33.BackColor = Color.Lime;
                    button34.BackColor = Color.White;
                }
                else
                {
                    button34.BackColor = Color.Red;
                    button33.BackColor = Color.White;
                }
            }

            startCoil = 17;  //  regBank.add(00017-24);   Отображение соостояния реле 17-24
            numCoils = 8;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));

            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                if (coilArr[0] == true)                             // XP7 4 PTT2 Танг. р.  regBank.add(17)
                {
                    button14.BackColor = Color.Lime;
                    button29.BackColor = Color.White;
                }
                else
                {
                    button29.BackColor = Color.Red;
                    button14.BackColor = Color.White;
                }
                if (coilArr[1] == true)                             // XP1 - 20  HangUp  DCD regBank.add(18)
                {
                    button19.BackColor = Color.Lime;
                    button26.BackColor = Color.White;
                }
                else
                {
                    button26.BackColor = Color.Red;
                    button19.BackColor = Color.White;
                }
                if (coilArr[2] == true)                             // J8-11     XP7 2 Sence  Танг. р. regBank.add(19)
                {
                    button58.BackColor = Color.Lime;
                    button72.BackColor = Color.White;
                }
                else
                {
                    button72.BackColor = Color.Red;
                    button58.BackColor = Color.White;
                }
                if (coilArr[3] == true)                             //  XP7 1 PTT1 Танг. р.  regBank.add(20)
                {
                    button10.BackColor = Color.Lime;
                    button23.BackColor = Color.White;
                }
                else
                {
                    button23.BackColor = Color.Red;
                    button10.BackColor = Color.White;
                }
                if (coilArr[4] == true)                             // XP2-2    Sence "Маг." 
                {
                    button60.BackColor = Color.Lime;
                    button76.BackColor = Color.White;
                }
                else
                {
                    button76.BackColor = Color.Red;
                    button60.BackColor = Color.White;
                }
                if (coilArr[5] == true)                             // XP5-3    Sence "ГГC."
                {
                    button35.BackColor = Color.Lime;
                    button36.BackColor = Color.White;
                }
                else
                {
                    button36.BackColor = Color.Red;
                    button35.BackColor = Color.White;
                }
                if (coilArr[6] == true)                             // XP3-3    Sence "ГГ-Радио1."
                {
                    button56.BackColor = Color.Lime;
                    button68.BackColor = Color.White;
                }
                else
                {
                    button68.BackColor = Color.Red;
                    button56.BackColor = Color.White;
                }
                if (coilArr[7] == true)                             // XP4-3    Sence "ГГ-Радио2."
                {
                    button55.BackColor = Color.Lime;
                    button67.BackColor = Color.White;
                }
                else
                {
                    button67.BackColor = Color.Red;
                    button55.BackColor = Color.White;
                }

            }

            startCoil = 25;  //  regBank.add(00001-12);   Отображение соостояния реле 25-32
            numCoils = 8;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));


            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {


                if (coilArr[0] == false)                            // XP1- 19 HaSs      Сенсор  подключения трубки       
                {
                    button57.BackColor = Color.Lime;
                    button70.BackColor = Color.White;
                }
                else
                {
                    button70.BackColor = Color.Red;
                    button57.BackColor = Color.White;
                }

                if (coilArr[1] == true)                             // XP1- 17 HaSPTT    CTS DSR вкл. 
                {
                    button16.BackColor = Color.Lime;
                    button20.BackColor = Color.White;
                }
                else
                {
                    button20.BackColor = Color.Red;
                    button16.BackColor = Color.White;
                }



                if (coilArr[2] == true)                             // XP1- 16 HeS2Rs    флаг подключения гарнитуры инструктора с 2 наушниками
                {
                    button61.BackColor = Color.Lime;
                    button78.BackColor = Color.White;
                }
                else
                {
                    button78.BackColor = Color.Red;
                    button61.BackColor = Color.White;
                }


                if (coilArr[3] == true)                             // XP1- 15 HeS2PTT   CTS вкл
                {
                    button28.BackColor = Color.Lime;
                    button17.BackColor = Color.White;
                }
                else
                {
                    button17.BackColor = Color.Red;
                    button28.BackColor = Color.White;
                }

                if (coilArr[4] == true)                             //    XP1- 13 HeS2Ls    флаг подключения гарнитуры инструктора 
                {
                    button62.BackColor = Color.Lime;
                    button77.BackColor = Color.White;
                }
                else
                {
                    button77.BackColor = Color.Red;
                    button62.BackColor = Color.White;
                }

                if (coilArr[5] == true)                             //    XP1- 6  HeS1PTT   CTS вкл
                {
                    button8.BackColor = Color.Lime;
                    button22.BackColor = Color.White;
                }
                else
                {
                    button22.BackColor = Color.Red;
                    button8.BackColor = Color.White;
                }

                if (coilArr[6] == true)                             //   XP1- 5  HeS1Rs    Флаг подкючения гарнитуры диспетчера с 2 наушниками
                {
                    button63.BackColor = Color.Lime;
                    button75.BackColor = Color.White;
                }
                else
                {
                    button75.BackColor = Color.Red;
                    button63.BackColor = Color.White;
                }

                if (coilArr[7] == true)                             //    XP1- 1  HeS1Ls    Флаг подкючения гарнитуры диспетчера
                {
                    button64.BackColor = Color.Lime;
                    button73.BackColor = Color.White;
                }
                else
                {
                    button73.BackColor = Color.Red;
                    button64.BackColor = Color.White;
                }


            }

            startCoil = 81;  // Флаг 
            numCoils = 4;
            res = myProtocol.readInputDiscretes(slave, startCoil, coilArr, numCoils);
            lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));

            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                if (coilArr[0] == false) // бит CTS - 1x81 
                {
                    label156.BackColor = Color.Red;
                    label156.Text = "1";
                }
                else
                {
                    label156.BackColor = Color.Lime;
                    label156.Text = "0";
                }
                if (coilArr[1] == false) // бит DSR - 1x82  
                {
                    label155.BackColor = Color.Red;
                    label155.Text = "1";
                }
                else
                {
                    label155.BackColor = Color.Lime;
                    label155.Text = "0";
                }
                if (coilArr[2] == false) // // бит DCD -  1x83
                {
                    label152.BackColor = Color.Red;
                    label152.Text = "1";
                }
                else
                {
                    label152.BackColor = Color.Lime;
                    label152.Text = "0";
                }

            }

            progressBar1.Value += 1;
            label114.Text = ("" + progressBar1.Value);
            if (progressBar1.Value == progressBar1.Maximum)
            {
                progressBar1.Value = 0;
            }

            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

                label83.Text = "";
                label83.Text = (label83.Text + readVals[0] + "." + readVals[1] + "." + readVals[2] + "   " + readVals[3] + ":" + readVals[4] + ":" + readVals[5]);
            }
            else
            {
                toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                toolStripStatusLabel1.BackColor = Color.Red;
                // Polltimer1.Enabled = false;
                timer_Mic_test.Enabled = false;
            }
            label80.Text = DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture);
            toolStripStatusLabel2.Text = ("Время : " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture));
        }
        private void timerCTS_Tick(object sender, EventArgs e)
        {

            // short[] writeVals = new short[32];
            short[] readVals = new short[124];
            int slave;
            int startRdReg;
            int numRdRegs;
            int startCoil;
            int numCoils;
            int i;
            int res;
            bool[] coilVals = new bool[64];
            bool[] coilArr = new bool[34];
            bool[] coilSensor = new bool[64];

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            startRdReg = 1;
            numRdRegs = 56;

            res = myProtocol.readInputRegisters(slave, startRdReg, readVals, numRdRegs);
            label78.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {

                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;


                //  Первый байт

                label30.Text = "";
                label31.Text = "";
                label32.Text = "";

                label33.Text = "";
                label34.Text = "";
                label35.Text = "";
                label36.Text = "";


                for (i = 7; i >= 0; i--)
                {
                    label30.Text = (label30.Text + (readVals[i] + "  "));
                    label31.Text = (label31.Text + (readVals[i + 8] + "  "));
                    label32.Text = (label32.Text + (readVals[i + 16] + "  "));

                    label33.Text = (label33.Text + (readVals[i + 24] + "  "));
                    label34.Text = (label34.Text + (readVals[i + 32] + "  "));
                    label35.Text = (label35.Text + (readVals[i + 40] + "  "));
                    label36.Text = (label36.Text + (readVals[i + 48] + "  "));
                }
                if (readVals[24] == 0) // 30024 флаг подключения ГГ Радио2
                {
                    label103.BackColor = Color.Red;
                    label103.Text = "0";
                }
                else
                {
                    label103.BackColor = Color.Lime;
                    label103.Text = "1";
                }
                if (readVals[25] == 0) // 30025 флаг подключения ГГ Радио1
                {
                    label104.BackColor = Color.Red;
                    label104.Text = "0";
                }
                else
                {
                    label104.BackColor = Color.Lime;
                    label104.Text = "1";
                }

                if (readVals[26] == 0) // 30026 флаг подключения трубки
                {
                    label105.BackColor = Color.Red;
                    label105.Text = "0";
                }
                else
                {
                    label105.BackColor = Color.Lime;
                    label105.Text = "1";
                }

                if (readVals[27] == 0)   // 30027 флаг подключения ручной тангенты
                {
                    label106.BackColor = Color.Red;
                    label106.Text = "0";
                }
                else
                {
                    label106.BackColor = Color.Lime;
                    label106.Text = "1";
                }

                if (readVals[28] == 0)  // 30028 флаг подключения педали
                {
                    label107.BackColor = Color.Red;
                    label107.Text = "0";
                }
                else
                {
                    label107.BackColor = Color.Lime;
                    label107.Text = "1";
                }

                if (readVals[40] == 0) // 30040  флаг подключения магнитофона
                {
                    label108.BackColor = Color.Red;
                    label108.Text = "0";
                }
                else
                {
                    label108.BackColor = Color.Lime;
                    label108.Text = "1";
                }

                if (readVals[41] == 0) // 30041  флаг подключения гарнитуры инструктора 2 наушниками
                {
                    label109.BackColor = Color.Red;
                    label109.Text = "0";
                }
                else
                {
                    label109.BackColor = Color.Lime;
                    label109.Text = "1";
                }

                if (readVals[42] == 0) // 30042  флаг подключения гарнитуры инструктора
                {
                    label110.BackColor = Color.Red;
                    label110.Text = "0";
                }
                else
                {
                    label110.BackColor = Color.Lime;
                    label110.Text = "1";
                }

                if (readVals[43] == 0) // 30043  флаг подключения гарнитуры диспетчера с 2 наушниками
                {
                    label111.BackColor = Color.Red;
                    label111.Text = "0";
                }
                else
                {
                    label111.BackColor = Color.Lime;
                    label111.Text = "1";
                }

                if (readVals[44] == 0) // 30044  флаг подключения гарнитуры диспетчера
                {
                    label112.BackColor = Color.Red;
                    label112.Text = "0";
                }
                else
                {
                    label112.BackColor = Color.Lime;
                    label112.Text = "1";
                }

                if (readVals[45] == 0) // 30045  флаг подключения микрофона XS1 - 6 Sence
                {
                    label113.BackColor = Color.Red;
                    label113.Text = "0";
                }
                else
                {
                    label113.BackColor = Color.Lime;
                    label113.Text = "1";
                }

                if (readVals[46] == 0) //  30046  флаг подключения ГГС
                {
                    label115.BackColor = Color.Red;
                    label115.Text = "0";
                }
                else
                {
                    label115.BackColor = Color.Lime;
                    label115.Text = "1";
                }


                if (readVals[47] == 0) // 30047   флаг выключения ГГС (Mute)
                {
                    label144.BackColor = Color.Red;
                    label144.Text = "0";
                }
                else
                {
                    label144.BackColor = Color.Lime;
                    label144.Text = "1";
                }

                if (readVals[48] == 0) // 30048   флаг радиопередачи
                {
                    label143.BackColor = Color.Red;
                    label143.Text = "0";
                }
                else
                {
                    label143.BackColor = Color.Lime;
                    label143.Text = "1";
                }

                if (readVals[49] == 0) // 30049   флаг управления микрофонами гарнитур
                {
                    label142.BackColor = Color.Red;
                    label142.Text = "0";
                }
                else
                {
                    label142.BackColor = Color.Lime;
                    label142.Text = "1";
                }
            }





            startCoil = 81;  // Флаг выполнения полного теста
            numCoils = 4;
            res = myProtocol.readInputDiscretes(slave, startCoil, coilArr, numCoils);
            lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));

            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                if (coilArr[0] == false) // бит CTS - 1x80   
                {
                    label156.BackColor = Color.Red;
                    label156.Text = "1";
                }
                else
                {
                    label156.BackColor = Color.Lime;
                    label156.Text = "0";
                }
                if (coilArr[1] == false) // бит DSR - 1x81   
                {
                    label155.BackColor = Color.Red;
                    label155.Text = "1";
                }
                else
                {
                    label155.BackColor = Color.Lime;
                    label155.Text = "0";
                }
                if (coilArr[2] == false) // // бит DCD -  1x82 
                {
                    label152.BackColor = Color.Red;
                    label152.Text = "1";
                }
                else
                {
                    label152.BackColor = Color.Lime;
                    label152.Text = "0";
                }

            }


            if ((res != BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                toolStripStatusLabel1.BackColor = Color.Red;
                //Polltimer1.Enabled = false;
                timer_Mic_test.Enabled = false;
            }



            progressBar1.Value += 1;
            label114.Text = ("" + progressBar1.Value);
            if (progressBar1.Value == progressBar1.Maximum)
            {
                progressBar1.Value = 0;
            }


            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

                label83.Text = "";
                label83.Text = (label83.Text + readVals[0] + "." + readVals[1] + "." + readVals[2] + "   " + readVals[3] + ":" + readVals[4] + ":" + readVals[5]);
            }


            else
            {
                toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                toolStripStatusLabel1.BackColor = Color.Red;
                // Polltimer1.Enabled = false;
                timer_Mic_test.Enabled = false;
                timer_byte_set.Enabled = false;
                timerCTS.Enabled = false;
                timerTestAll.Enabled = false;



            }


            label80.Text = DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture);
            toolStripStatusLabel2.Text = ("Время : " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture));
        }


        #endregion

        private void cmdOpenTCP_Click(object sender, EventArgs e)
        {
            //
            // First we must instantiate class if we haven't done so already
            //
            if ((myProtocol == null))
            {
                try
                {
                    if ((cmbTcpProtocol.SelectedIndex == 0))
                        myProtocol = new MbusTcpMasterProtocol();
                    else
                        myProtocol = new MbusRtuOverTcpMasterProtocol();
                }
                catch (OutOfMemoryException ex)
                {
                    label78.Text = ("Не удалось создать экземпляр класса серийного протокола! Ошибка была " + ex.Message);
                    return;
                }
            }
            else // already instantiated, close protocol and reinstantiate
            {
                if (myProtocol.isOpen())
                    myProtocol.closeProtocol();
                myProtocol = null;
                try
                {
                    if ((cmbTcpProtocol.SelectedIndex == 0))
                        myProtocol = new MbusTcpMasterProtocol();
                    else
                        myProtocol = new MbusRtuOverTcpMasterProtocol();
                }
                catch (OutOfMemoryException ex)
                {
                    label78.Text = ("Не удалось создать экземпляр класса серийного протокола! Ошибка была " + ex.Message);
                    return;
                }
            }
            //
            // Here we configure the protocol
            //
            int retryCnt;
            int pollDelay;
            int timeOut;
            int tcpPort;
            int res;
            try
            {
                retryCnt = int.Parse(cmbRetry.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                retryCnt = 0;
            }
            try
            {
                pollDelay = int.Parse(txtPollDelay.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                pollDelay = 0;
            }
            try
            {
                timeOut = int.Parse(txtTimeout.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                timeOut = 1000;
            }
            try
            {
                tcpPort = int.Parse(txtTCPPort.Text, CultureInfo.CurrentCulture);
            }
            catch (Exception)
            {
                tcpPort = 502;
            }
            myProtocol.timeout = timeOut;
            myProtocol.retryCnt = retryCnt;
            myProtocol.pollDelay = pollDelay;
            // Note: The following cast is required as the myProtocol object is declared
            // as the superclass of MbusTcpMasterProtocol. That way myProtocol can
            // represent different protocol types.
            ((MbusTcpMasterProtocol)myProtocol).port = (short)tcpPort;
            res = ((MbusTcpMasterProtocol)myProtocol).openProtocol(txtHostName.Text);
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                label78.Text = ("Modbus/TCP port opened successfully with parameters: " + (txtHostName.Text + (", TCP port " + tcpPort)));
                button6.Enabled = true;
                Polltimer1.Enabled = true;
            }
            else
            {
                label78.Text = ("Could not open protocol, error was: " + BusProtocolErrors.getBusProtocolErrorText(res));
            }
        }

        private void cmdExecute_Click(object sender, EventArgs e)
        {
            short[] writeVals = new short[125];
            short[] readVals = new short[125];
            int slave;
            int startWrReg;
            int numWrRegs;
            int startRdReg;
            int numRdRegs;
            int i;
            int res;
            int startCoil;
            int numCoils;
            bool[] coilVals = new bool[2000];
            try
            {
                try
                {
                    slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                }
                catch (Exception)
                {
                    slave = 1;
                }
                try
                {
                    startCoil = int.Parse(txtStartCoil.Text, CultureInfo.CurrentCulture);
                }
                catch (Exception)
                {
                    startCoil = 1;
                }
                try
                {
                    numCoils = int.Parse(txtNumCoils.Text, CultureInfo.CurrentCulture);
                }
                catch (Exception)
                {
                    numCoils = 1;
                }
                try
                {
                    startRdReg = int.Parse(txtStartRdReg.Text, CultureInfo.CurrentCulture);
                }
                catch (Exception)
                {
                    startRdReg = 1;
                }
                try
                {
                    startWrReg = int.Parse(txtStartWrReg.Text, CultureInfo.CurrentCulture);
                }
                catch (Exception)
                {
                    startWrReg = 1;
                }
                try
                {
                    numWrRegs = int.Parse(txtNumWrRegs.Text, CultureInfo.CurrentCulture);
                }
                catch (Exception)
                {
                    numWrRegs = 1;
                }
                try
                {
                    numRdRegs = int.Parse(txtNumRdRegs.Text, CultureInfo.CurrentCulture);
                }
                catch (Exception)
                {
                    numRdRegs = 1;
                }
                try
                {
                    writeVals[0] = Int16.Parse(txtWriteVal1.Text, CultureInfo.CurrentCulture);
                    writeVals[1] = Int16.Parse(txtWriteVal2.Text, CultureInfo.CurrentCulture);
                    writeVals[2] = Int16.Parse(txtWriteVal3.Text, CultureInfo.CurrentCulture);
                    writeVals[3] = Int16.Parse(txtWriteVal4.Text, CultureInfo.CurrentCulture);
                    writeVals[4] = Int16.Parse(txtWriteVal5.Text, CultureInfo.CurrentCulture);
                    writeVals[5] = Int16.Parse(txtWriteVal6.Text, CultureInfo.CurrentCulture);
                    writeVals[6] = Int16.Parse(txtWriteVal7.Text, CultureInfo.CurrentCulture);
                    writeVals[7] = Int16.Parse(txtWriteVal8.Text, CultureInfo.CurrentCulture);
                    coilVals[0] = (writeVals[0] != 0);
                    coilVals[1] = (writeVals[1] != 0);
                    coilVals[2] = (writeVals[2] != 0);
                    coilVals[3] = (writeVals[3] != 0);
                    coilVals[4] = (writeVals[4] != 0);
                    coilVals[5] = (writeVals[5] != 0);
                    coilVals[6] = (writeVals[6] != 0);
                    coilVals[7] = (writeVals[7] != 0);
                }
                catch (Exception)
                {
                }
                switch (cmbCommand.SelectedIndex)
                {
                    //
                    // Read Holding Registers
                    //
                    case 0:
                        res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);
                        lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                        if ((res == BusProtocolErrors.FTALK_SUCCESS))
                        {
                            lblReadValues.Text = "";
                            for (i = 0; (i <= (numRdRegs - 1)); i++)
                            {
                                lblReadValues.Text = (lblReadValues.Text + (readVals[i] + "  "));
                            }
                        }
                        break;
                    //
                    // Read Input Registers
                    //
                    case 1:
                        res = myProtocol.readInputRegisters(slave, startRdReg, readVals, numRdRegs);
                        lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                        if ((res == BusProtocolErrors.FTALK_SUCCESS))
                        {
                            lblReadValues.Text = "";
                            for (i = 0; (i <= (numRdRegs - 1)); i++)
                            {
                                lblReadValues.Text = (lblReadValues.Text + (readVals[i] + "  "));
                            }
                        }
                        break;
                    //
                    // Read Coils
                    //
                    case 2:
                        res = myProtocol.readCoils(slave, startCoil, coilVals, numCoils);
                        lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                        if ((res == BusProtocolErrors.FTALK_SUCCESS))
                        {
                            lblReadValues.Text = "";
                            for (i = 0; (i <= (numCoils - 1)); i++)
                            {
                                if (coilVals[i])
                                    lblReadValues.Text = (lblReadValues.Text + "1  ");
                                else
                                    lblReadValues.Text = (lblReadValues.Text + "0  ");
                            }
                        }
                        break;
                    //
                    // Write Coils
                    //
                    case 3:
                        res = myProtocol.forceMultipleCoils(slave, startCoil, coilVals, numCoils);
                        lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                        break;
                    //
                    // Write Holding Registers
                    //
                    case 4:
                        res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
                        lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                        break;
                    //
                    // Write Single Registers
                    //
                    case 5:
                        res = myProtocol.writeSingleRegister(slave, startWrReg, writeVals[0]);
                        lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                        break;
                    //
                    // Read/Write Registers
                    //
                    case 6:
                        res = myProtocol.readWriteRegisters(slave, startRdReg, readVals, numRdRegs, startWrReg, writeVals, numWrRegs);
                        lblResult2.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
                        break;
                }
            }
            catch (Exception ex)
            {
                lblResult2.Text = ("Exception occured: " + ex.Message);
            }
        }

        private void label24_Click(object sender, EventArgs e)
        {

        }

        private void label25_Click(object sender, EventArgs e)
        {

        }

        private void cmbBaudRate_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void tabPage3_Click(object sender, EventArgs e)
        {

        }

        private void dateTimePicker2_ValueChanged(object sender, EventArgs e)
        {

        }

        private void label75_Click(object sender, EventArgs e)
        {

        }

        private void label76_Click(object sender, EventArgs e)
        {

        }

        private void button3_Click(object sender, EventArgs e)                         // Записать системное время
        {

            ushort[] writeVals = new ushort[20];
            bool[] coilVals = new bool[2];
            int slave;                           //
            int res;
            int startWrReg;
            int numWrRegs;   //

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            int numVal = -1;

            //    string command = DateTime.Parse(label80.Text, CultureInfo.CurrentCulture).ToString("ddMMyyyyHHmmss", CultureInfo.CurrentCulture);

            string command = label80.Text;
            numVal = Convert.ToInt32(command.Substring(0, 2), CultureInfo.CurrentCulture);
            writeVals[0] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(3, 2), CultureInfo.CurrentCulture);
            writeVals[1] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(6, 4), CultureInfo.CurrentCulture);
            writeVals[2] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(11, 2), CultureInfo.CurrentCulture);
            writeVals[3] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(14, 2), CultureInfo.CurrentCulture);
            writeVals[4] = (ushort)numVal;   // 

            startWrReg = 52;
            numWrRegs = 6;   //

            res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 14);                          // Записать системное время
        }

        private void button4_Click(object sender, EventArgs e)                         // Записать пользовательское время
        {

            ushort[] writeVals = new ushort[20];
            bool[] coilVals = new bool[2];
            int slave;                           //
            int res;
            int startWrReg;
            int numWrRegs;   //

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            int numVal = -1;

            string command = dateTimePicker1.Value.ToString("ddMMyyyyHHmmss", CultureInfo.CurrentCulture);

            numVal = Convert.ToInt32(command.Substring(0, 2), CultureInfo.CurrentCulture);
            writeVals[0] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(2, 2), CultureInfo.CurrentCulture);
            writeVals[1] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(4, 4), CultureInfo.CurrentCulture);
            writeVals[2] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(8, 2), CultureInfo.CurrentCulture);
            writeVals[3] = (ushort)numVal;   // 
            numVal = Convert.ToInt32(command.Substring(10, 2), CultureInfo.CurrentCulture);
            writeVals[4] = (ushort)numVal;   // 

            startWrReg = 52;
            numWrRegs = 6;   //
            res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 14);                       // Записать новое время пользователя
        }

        private void dateTimePicker1_ValueChanged(object sender, EventArgs e)
        {

        }

        private void button5_Click(object sender, EventArgs e)                         // Закрыть сериал и протокол
        {

            // Close protocol and serial port
            myProtocol.closeProtocol();
            //    // Indicate result on status line
            lblResult.Text = "Протокол закрыт";
            //    // Disable button controls
            button5.Enabled = false;
            cmdOpenSerial.Enabled = true;
            Polltimer1.Enabled = false;
            toolStripStatusLabel1.Text = "  MODBUS ЗАКРЫТ   ";
            toolStripStatusLabel1.BackColor = Color.Red;


        }

        private void button6_Click(object sender, EventArgs e)                        // Закрыть TCP и протокол
        {
            // Close protocol and serial port
            myProtocol.closeProtocol();
            //    // Indicate result on status line
            lblResult.Text = "Протокол закрыт";
            //    // Disable button controls
            button6.Enabled = false;
            cmdOpenTCP.Enabled = true;
            Polltimer1.Enabled = false;
            toolStripStatusLabel1.Text = "  MODBUS ЗАКРЫТ   ";
            toolStripStatusLabel1.BackColor = Color.Red;


        }

        private void label48_Click(object sender, EventArgs e)
        {

        }

        private void tabPage6_Click(object sender, EventArgs e)
        {

        }

        // Label
        private void label88_Click(object sender, EventArgs e)
        {

        }
        private void label92_Click_1(object sender, EventArgs e)
        {

        }
        private void label49_Click(object sender, EventArgs e)
        {

        }
        private void label104_Click(object sender, EventArgs e)
        {

        }
        private void label51_Click(object sender, EventArgs e)
        {

        }
        private void label118_Click(object sender, EventArgs e)
        {

        }

        private void label116_Click(object sender, EventArgs e)
        {

        }

        private void label52_Click(object sender, EventArgs e)
        {

        }

        private void label50_Click(object sender, EventArgs e)
        {

        }

        private void label59_Click(object sender, EventArgs e)
        {

        }

        private void label68_Click(object sender, EventArgs e)
        {

        }

        private void label62_Click(object sender, EventArgs e)
        {

        }

        private void label159_Click(object sender, EventArgs e)
        {

        }

        private void label144_Click(object sender, EventArgs e)
        {

        }

        private void label103_Click(object sender, EventArgs e)
        {

        }
        private void label73_Click(object sender, EventArgs e)
        {

        }

        private void label102_Click(object sender, EventArgs e)
        {

        }

        private void label90_Click(object sender, EventArgs e)
        {

        }

        private void label97_Click(object sender, EventArgs e)
        {

        }

        private void label142_Click(object sender, EventArgs e)
        {

        }

        private void label143_Click(object sender, EventArgs e)
        {

        }

        private void label147_Click(object sender, EventArgs e)
        {

        }

        private void label156_Click(object sender, EventArgs e)
        {

        }

        private void label155_Click(object sender, EventArgs e)
        {

        }

        private void label27_Click(object sender, EventArgs e)
        {

        }

        private void label133_Click(object sender, EventArgs e)
        {

        }

        private void label158_Click(object sender, EventArgs e)
        {

        }

        private void label153_Click(object sender, EventArgs e)
        {

        }

        private void label129_Click(object sender, EventArgs e)
        {

        }

        private void label130_Click(object sender, EventArgs e)
        {

        }

        private void label127_Click(object sender, EventArgs e)
        {

        }

        private void label105_Click(object sender, EventArgs e)
        {

        }

        private void label63_Click(object sender, EventArgs e)
        {

        }

        private void label45_Click(object sender, EventArgs e)
        {

        }

        private void label47_Click(object sender, EventArgs e)
        {

        }




        #region Button test
        private void tabPage5_Click(object sender, EventArgs e)
        {
            if (TabControl1.SelectedIndex == 2)
            {
                //Polltimer1.Enabled = false;                                              // Запретить опрос состояния
                //timer_Mic_test.Enabled = false;                                            // Запретить тест микрофона
                //timerCTS.Enabled = false;
                //timerTestAll.Enabled = false;


                //bool[] coilVals = new bool[200];
                //slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
                //progressBar1.Value = 0;

                //button32.BackColor = Color.Lime;                              //Изменение цвета кнопок
                //button31.BackColor = Color.LightSalmon;
                //label102.Text = "Выполняется контроль состояния сенсоров";
                //label102.ForeColor = Color.DarkOliveGreen;

                ////startCoil = 33; // Запустить тест CTS
                ////res = myProtocol.writeCoil(slave, startCoil, true); 
                //timer_byte_set.Enabled = true;        // Включить контроль состояния модуля Камертон            

            }
        }

        private void button32_Click(object sender, EventArgs e)                       //Старт теста "Байты обмена с Камертон"
        {
            Polltimer1.Enabled = false;                                            // Запретить опрос состояния
            timer_Mic_test.Enabled = false;                                            // Запретить тест микрофона
            timerCTS.Enabled = false;
            timerTestAll.Enabled = false;


            bool[] coilVals = new bool[200];
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
            progressBar1.Value = 0;

            button32.BackColor = Color.Lime;                                           //Изменение цвета кнопок
            button31.BackColor = Color.LightSalmon;
            label102.Text = "Выполняется контроль состояния сенсоров";
            label102.ForeColor = Color.DarkOliveGreen;

            //startCoil = 33; // Запустить тест CTS
            //res = myProtocol.writeCoil(slave, startCoil, true); 
            timer_byte_set.Enabled = true;                                           // Включить контроль состояния модуля Камертон            

        }

        private void button31_Click(object sender, EventArgs e)                       //Останов теста "Байты обмена с Камертон"
        {
            startCoil = 33; // Остановить тест CTS

            res = myProtocol.writeCoil(slave, startCoil, false);
            button31.BackColor = Color.Red;
            button32.BackColor = Color.White;
            label102.Text = "Контроль состояния сенсоров ОСТАНОВЛЕН";
            label102.ForeColor = Color.Red;
            progressBar1.Value = 0;
            timer_byte_set.Enabled = false;
            Polltimer1.Enabled = true;
        }

        private void progressBar1_Click(object sender, EventArgs e)
        {

        }


        //****************  Включение реле ************************************************
        private void button37_Click(object sender, EventArgs e)                      // ВКЛ Реле RL0
        {
            startCoil = 1; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button48_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL0
        {

            startCoil = 1; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button40_Click(object sender, EventArgs e)                      // ВКЛ Реле RL1
        {
            startCoil = 2; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button53_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL1
        {
            startCoil = 2; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button44_Click(object sender, EventArgs e)                      // ВКЛ Реле RL2
        {
            startCoil = 3; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button79_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL2
        {
            startCoil = 3; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button49_Click(object sender, EventArgs e)                      // ВКЛ Реле RL3
        {
            startCoil = 4; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button66_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL3
        {
            startCoil = 4; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button38_Click(object sender, EventArgs e)                      // ВКЛ Реле RL4
        {
            startCoil = 5; // Микрофон инструктора включить
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button52_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL4
        {
            startCoil = 5; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button71_Click(object sender, EventArgs e)                      // ВКЛ Реле RL5
        {
            startCoil = 6; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button47_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL5
        {
            startCoil = 6; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button69_Click(object sender, EventArgs e)                      // ВКЛ Реле RL6
        {
            startCoil = 7; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button42_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL6
        {
            startCoil = 7; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button51_Click(object sender, EventArgs e)                      // ВКЛ Реле RL7
        {
            startCoil = 8; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button45_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL7
        {
            startCoil = 8; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button46_Click(object sender, EventArgs e)                      // ВКЛ Реле RL8
        {
            startCoil = 9; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button50_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL8
        {
            startCoil = 9; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button27_Click(object sender, EventArgs e)                      // ВКЛ Реле RL9
        {
            startCoil = 10; // Микрофон диспетчера включить
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button30_Click(object sender, EventArgs e)                      // ВЫКЛ Реле RL9
        {
            startCoil = 10; // Микрофон диспетчера отключить
            res = myProtocol.writeCoil(slave, startCoil, false);
        }


        //**********************************************************************************************




        private void button55_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор ГГ-Радио2
        {
            startCoil = 24;                                                          // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button67_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор ГГ-Радио2
        {
            startCoil = 24;                                                           // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button56_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор ГГ-Радио1
        {
            startCoil = 23; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }
        private void button68_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор ГГ-Радио1
        {
            startCoil = 23; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button57_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ подключения трубки    XP1- 19 HaSs  
        {
            startCoil = 25; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }
        private void button70_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ подключения трубки    XP1- 19 HaSs  
        {
            startCoil = 25; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button58_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор  Танг.р 
        {
            startCoil = 19; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button72_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор Танг.р 
        {
            startCoil = 19; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button59_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор Танг.н
        {
            startCoil = 13; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button74_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор Танг.н
        {
            startCoil = 13; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button60_Click(object sender, EventArgs e)                      // Кнопка  ВКЛ Сенсор  Маг.
        {
            startCoil = 21; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button76_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор  Маг.
        {
            startCoil = 21; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button61_Click(object sender, EventArgs e)                      // 7 Кнопка  ВКЛ   XP1- 16 HeS2Rs Сенсор подключения гарнитуры инструктора с 2 наушниками
        {
            startCoil = 27; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button78_Click(object sender, EventArgs e)                      // 7 Кнопка  ОТКЛ  XP1- 16 HeS2Rs  Сенсор подключения гарнитуры инструктора с 2 наушниками
        {
            startCoil = 27; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button62_Click(object sender, EventArgs e)                      // ВКЛ XP1- 13 HeS2Ls Кнопка  ВКЛ флаг подключения гарнитуры инструктора 
        {
            startCoil = 29; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button77_Click(object sender, EventArgs e)                      // ОТКЛ XP1- 13 HeS2Ls Кнопка  ОТКЛ флаг подключения гарнитуры инструктора  
        {
            startCoil = 29; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button63_Click(object sender, EventArgs e)                      //  XP1- 5  HeS1Rs Кнопка  ВКЛ Флаг подключения гарнитуры диспетчера с 2 наушниками
        {
            startCoil = 31; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button75_Click(object sender, EventArgs e)                      //  XP1- 5  HeS1Rs Кнопка  ОТКЛ Флаг подключения гарнитуры диспетчера с 2 наушниками
        {
            startCoil = 31; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button64_Click(object sender, EventArgs e)                      // XP1- 1  HeS1Ls  Кнопка  ВКЛ  Флаг подключения гарнитуры диспетчера
        {
            startCoil = 32; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button73_Click(object sender, EventArgs e)                      // XP1- 1  HeS1Ls  Кнопка  ОТКЛ Флаг подключения гарнитуры диспетчера
        {
            startCoil = 32; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button33_Click(object sender, EventArgs e)                      //  Кнопка  ВКЛ Сенсор Мик.
        {
            startCoil = 16; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button34_Click(object sender, EventArgs e)                      // Кнопка  ОТКЛ Сенсор Мик.
        {
            startCoil = 16; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button35_Click(object sender, EventArgs e)                      //  Кнопка  ВКЛ Сенсор ГГС
        {
            startCoil = 22; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button36_Click(object sender, EventArgs e)                      //  Кнопка  ОТКЛ Сенсор ГГС
        {
            startCoil = 22; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }


        //********************** Вторая колонка кнопок*****************************
        private void label28_Click(object sender, EventArgs e) // 
        {
        }

        private void button7_Click(object sender, EventArgs e)                       // ВКл . XP8 - 1  PTT Мик
        {
            startCoil = 15; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button18_Click(object sender, EventArgs e)                      // ОТКЛ XP8 - 1  PTT Мик
        {
            startCoil = 15; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button10_Click(object sender, EventArgs e)                      // ВКЛ   XP7 1 PTT1 Танг. р.
        {
            startCoil = 20; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button23_Click(object sender, EventArgs e)                      // ОТКЛ   XP7 1 PTT1 Танг. р.
        {
            startCoil = 20; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button14_Click(object sender, EventArgs e)                      // ВКЛ  XP7 4 PTT2 Танг. р.
        {
            startCoil = 17; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button29_Click(object sender, EventArgs e)                      // ОТКЛ  XP7 4 PTT2 Танг. р.
        {
            startCoil = 17; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button19_Click(object sender, EventArgs e)                      // ВКЛ XP1 - 20  HangUp  DCD
        {
            startCoil = 18; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button26_Click(object sender, EventArgs e)                      // ОТКЛ XP1 - 20  HangUp  DCD
        {
            startCoil = 18; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button8_Click(object sender, EventArgs e)                       // ВКЛ XP1- 6  HeS1PTT   CTS вкл
        {
            startCoil = 30; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button22_Click(object sender, EventArgs e)                      // ОТКЛ  XP1- 6  HeS1PTT   CTS вкл
        {
            startCoil = 30; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button28_Click(object sender, EventArgs e)                      //ВКЛ  XP1- 15 HeS2PTT   CTS вкл
        {
            startCoil = 28; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button17_Click(object sender, EventArgs e)                      // ОТКЛ  XP1- 15 HeS2PTT   CTS вкл
        {
            startCoil = 28; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button16_Click(object sender, EventArgs e)                      //ВКЛ  XP1- 17 HaSPTT    CTS DSR вкл.
        {
            startCoil = 26; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button20_Click(object sender, EventArgs e)                      //ОТКЛ  XP1- 17 HaSPTT    CTS DSR вкл.
        {
            startCoil = 26; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }


        private void button39_Click(object sender, EventArgs e)                      // ВКЛ XP8-1 РТТ Танг. н.
        {
            startCoil = 14; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, true);
        }

        private void button41_Click(object sender, EventArgs e)                      //  ОТКЛ XP8-1  РТТ Танг. н.
        {
            startCoil = 14; // Управление сенсорами
            res = myProtocol.writeCoil(slave, startCoil, false);
        }

        private void button43_Click(object sender, EventArgs e)
        {

        }

        //**********************************************************************************************
        #endregion

        #region Test all
        // для вызова тестов необходимо отправить по адресу 120  в контроллер номер теста  (1-12)
        // далее отправить команду (true) вызова  теста по адресу 38

        private void sensor_off()// 
        {
            ushort[] writeVals = new ushort[2];
            short[] readVals = new short[124];
            bool[] coilArr = new bool[64];
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 1); // Отключить все сенсоры
            //startWrReg = 121;                                                                   // 
            //res = myProtocol.writeSingleRegister(slave, startWrReg, 0);               // Команда на запись  счетчиков отправлена
            textBox7.Text += ("Команда на отключение сенсоров отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(700);


            // Новый фрагмент чтения регистров 40001-40007

            int startRdReg;
            int numRdRegs;
            int i;
            bool[] coilVals = new bool[64];
            bool[] coilSensor = new bool[64];

            //*************************  Получить данные состояния модуля Камертон ************************************

            Int64 binaryHolder;
            string binaryResult = "";
            int decimalNum;
            bool[] Dec_bin = new bool[64];
            startRdReg = 1;
            numRdRegs = 7;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);    // 03  Считать число из регистров по адресу  40000 -49999
            label78.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

                for (int bite_x = 0; bite_x < 7; bite_x++)
                {
                    decimalNum = readVals[bite_x];
                    while (decimalNum > 0)
                    {
                        binaryHolder = decimalNum % 2;
                        binaryResult += binaryHolder;
                        decimalNum = decimalNum / 2;
                    }

                    int len_str = binaryResult.Length;

                    while (len_str < 8)
                    {
                        binaryResult += 0;
                        len_str++;
                    }

                    //****************** Перемена битов ***************************
                    //binaryArray = binaryResult.ToCharArray();
                    //Array.Reverse(binaryArray);
                    //binaryResult = new string(binaryArray);
                    //*************************************************************

                    for (i = 0; i < 8; i++)                         // 
                    {
                        if (binaryResult[i] == '1')
                        {
                            Dec_bin[i + (8 * bite_x)] = true;
                        }
                        else
                        {
                            Dec_bin[i + (8 * bite_x)] = false;
                        }
                    }
                    binaryResult = "";
                }

            }
            //if (Dec_bin[24] == false) // флаг подключения ГГ Радио2
            //{
            //    textBox8.Text += ("Сенсор ГГ Радио2 не отключился" + "\r\n");
            //    textBox8.Refresh();
            //}

            //if (Dec_bin[25] == false) // флаг подключения ГГ Радио1
            //{
            //    textBox8.Text += ("Сенсор ГГ Радио1 не отключился" + "\r\n");
            //    textBox8.Refresh();
            //}
/*
            if (Dec_bin[26] == true) //  флаг подключения трубки
            {
                //      textBox8.Text += ("Сенсор подключения трубки не отключился" + "\r\n");
                textBox8.Refresh();
            }



            if (Dec_bin[27] == true)   // флаг подключения ручной тангенты
            {
                //     textBox8.Text += ("Сенсор подключения ручной тангенты не отключился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[28] == true)  // флаг подключения педали
            {
                //      textBox8.Text += ("Сенсор подключения педали не отключился" + "\r\n");
                textBox8.Refresh();
            }

            */
            //if (Dec_bin[40] == true) //  флаг подключения магнитофона
            //{
            //    textBox8.Text += ("Сенсор подключения магнитофона не отключился" + "\r\n");
            //    textBox8.Refresh();
            //}

/*
            if (Dec_bin[41] == true) //  флаг подключения гарнитуры инструктора 2 наушниками
            {
                //      textBox8.Text += ("Сенсор отключения гарнитуры инструктора 2 наушниками не отключился" + "\r\n");
                textBox8.Refresh();
            }

            if (Dec_bin[42] == true) //   флаг подключения гарнитуры инструктора
            {
                //     textBox8.Text += ("Сенсор подключения гарнитуры инструктора не отключился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[43] == true) //  флаг подключения гарнитуры диспетчера с 2 наушниками
            {
                //    textBox8.Text += ("Сенсор подключения гарнитуры диспетчера с 2 наушниками не отключился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[44] == true) //  флаг подключения гарнитуры диспетчера
            {
                //     textBox8.Text += ("Сенсор подключения гарнитуры диспетчера не отключился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[45] == true) //  флаг подключения микрофона XS1 - 6 Sence  
            {
                //     textBox8.Text += ("Сенсор подключения микрофона не отключился" + "\r\n");
                textBox8.Refresh();
            }

            */
            //if (Dec_bin[46] == true) // флаг подключения ГГС
            //{
            //    textBox8.Text += ("Сенсор подключения ГГС не отключился" + "\r\n");
            //    textBox8.Refresh();
            //}
            //if (Dec_bin[52] == true) //  флаг подключения гарнитуры диспетчера
            //{
            //    textBox8.Text += ("Микрофон гарнитуры инструктора не отключился" + "\r\n");
            //    textBox8.Refresh();
            //}


            //if (Dec_bin[54] == true) //  флаг подключения микрофона XS1 - 6 Sence  
            //{
            //    textBox8.Text += ("Микрофон гарнитуры диспетчера не отключился" + "\r\n");
            //    textBox8.Refresh();
            //}


            test_end();
        }
        private void sensor_on()
        {
            ushort[] writeVals = new ushort[2];

            short[] readVals = new short[124];
            bool[] coilArr = new bool[64];
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 2); // Включить все сенсоры

            textBox7.Text += ("Команда на включение сенсоров отправлена" + "\r\n");
            textBox7.Refresh();

            Thread.Sleep(600);

            //  фрагмент чтения регистров 40001-40007 состояния "Камертон"

            int startRdReg;
            int numRdRegs;
            int i;
            bool[] coilVals = new bool[64];
            bool[] coilSensor = new bool[64];

            //*************************  Получить данные состояния модуля Камертон ************************************

            Int64 binaryHolder;
            string binaryResult = "";
            int decimalNum;
            bool[] Dec_bin = new bool[64];
            startRdReg = 1;
            numRdRegs = 7;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);    // 03  Считать число из регистров по адресу  40000 -49999
            label78.Text = ("Результат: " + (BusProtocolErrors.getBusProtocolErrorText(res) + "\r\n"));
            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;

                for (int bite_x = 0; bite_x < 7; bite_x++)
                {
                    decimalNum = readVals[bite_x];
                    while (decimalNum > 0)
                    {
                        binaryHolder = decimalNum % 2;
                        binaryResult += binaryHolder;
                        decimalNum = decimalNum / 2;
                    }

                    int len_str = binaryResult.Length;

                    while (len_str < 8)
                    {
                        binaryResult += 0;
                        len_str++;
                    }

                    //****************** Перемена битов ***************************
                    //binaryArray = binaryResult.ToCharArray();
                    //Array.Reverse(binaryArray);
                    //binaryResult = new string(binaryArray);
                    //*************************************************************

                    for (i = 0; i < 8; i++)                         // 
                    {
                        if (binaryResult[i] == '1')
                        {
                            Dec_bin[i + (8 * bite_x)] = true;
                        }
                        else
                        {
                            Dec_bin[i + (8 * bite_x)] = false;
                        }
                    }
                    binaryResult = "";
                }

            }
            //if (Dec_bin[24] == false) // флаг подключения ГГ Радио2
            //{
            //    textBox8.Text += ("Сенсор ГГ Радио2 не включился" + "\r\n");
            //    textBox8.Refresh();
            //}

            //if (Dec_bin[25] == false) // флаг подключения ГГ Радио1
            //{
            //    textBox8.Text += ("Сенсор ГГ Радио1 не включился" + "\r\n");
            //    textBox8.Refresh();
            //}
/*
            if (Dec_bin[26] == false) //  флаг подключения трубки
            {
                //    textBox8.Text += ("Сенсор подключения трубки не включился" + "\r\n");
                textBox8.Refresh();
            }



            if (Dec_bin[27] == false)   // флаг подключения ручной тангенты
            {
                //     textBox8.Text += ("Сенсор подключения ручной тангенты не включился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[28] == false)  // флаг подключения педали
            {
                //    textBox8.Text += ("Сенсор подключения педали не включился" + "\r\n");
                textBox8.Refresh();
            }
            */

            //if (Dec_bin[40] == false) //  флаг подключения магнитофона
            //{
            //    textBox8.Text += ("Сенсор подключения магнитофона не включился" + "\r\n");
            //    textBox8.Refresh();
            //}

/*
            if (Dec_bin[41] == false) //  флаг подключения гарнитуры инструктора 2 наушниками
            {
                //    textBox8.Text += ("Сенсор отключения гарнитуры инструктора 2 наушниками не включился" + "\r\n");
                textBox8.Refresh();
            }

            if (Dec_bin[42] == false) //   флаг подключения гарнитуры инструктора
            {
                //    textBox8.Text += ("Сенсор подключения гарнитуры инструктора не включился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[43] == false) //  флаг подключения гарнитуры диспетчера с 2 наушниками
            {
                //    textBox8.Text += ("Сенсор подключения гарнитуры диспетчера с 2 наушниками не включился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[44] == false) //  флаг подключения гарнитуры диспетчера
            {
                //    textBox8.Text += ("Сенсор подключения гарнитуры диспетчера не включился" + "\r\n");
                textBox8.Refresh();
            }


            if (Dec_bin[45] == false) //  флаг подключения микрофона XS1 - 6 Sence  
            {
                //    textBox8.Text += ("Сенсор подключения микрофона не включился" + "\r\n");
                textBox8.Refresh();
            }
            */
            //if (Dec_bin[46] == false) // флаг подключения ГГС
            //{
            //    textBox8.Text += ("Сенсор подключения ГГС не включился" + "\r\n");
            //    textBox8.Refresh();
            //}
            //if (Dec_bin[52] == false) //  флаг подключения гарнитуры диспетчера
            //{
            //    textBox8.Text += ("Микрофон гарнитуры инструктора не включился" + "\r\n");
            //    textBox8.Refresh();
            //}


            //if (Dec_bin[54] == false) //  флаг подключения микрофона XS1 - 6 Sence  
            //{
            //    textBox8.Text += ("Микрофон гарнитуры диспетчера не включился" + "\r\n");
            //    textBox8.Refresh();
            //}


            test_end();
        }
        private void test_instruktora()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[2];
            short[] readVals = new short[125];
            bool[] coilVals = new bool[120];
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            startWrReg = 120;  // В 40120 ячейке хранится номер теста. Эту ячейку применяет test_switch() Arduino
            res = myProtocol.writeSingleRegister(slave, startWrReg, 3); // Отключить все сенсоры
            textBox7.Text += ("Команда на проверку 'Гарнитура Инструктора' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);

            test_end();
        }
        private void test_dispetchera()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];

            startWrReg = 120;  // В 40120 ячейке хранится номер теста. Эту ячейку применяет test_switch() Arduino
            res = myProtocol.writeSingleRegister(slave, startWrReg, 4); // Отключить все сенсоры
            startCoil = 38; // Запустить полный тест , адрес в контроллере 37
            res = myProtocol.writeCoil(slave, startCoil, true);
            textBox7.Text += ("Команда на проверку 'Гарнитура Диспетчера' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        private void test_MTT()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 5); // Отключить все сенсоры
            textBox7.Text += ("Команда на проверку 'МТТ' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        private void test_tangR()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 6); // 
            textBox7.Text += ("Команда на проверку 'Тангента ручная' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        private void test_tangN()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 7); // Отключить все сенсоры
            textBox7.Text += ("Команда на проверку 'Тангента ножная' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        private void testGGS()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 8); // Отключить все сенсоры
            textBox7.Text += ("Команда на ГГС отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        private void test_GG_Radio1()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 9); // Отключить все сенсоры
            textBox7.Text += ("Команда на проверку 'ГГ-Радио1' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        private void test_GG_Radio2()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 10); // Отключить все сенсоры
            textBox7.Text += ("Команда на проверку 'ГГ-Радио2' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        private void test_mikrophon()
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 11); // Отключить все сенсоры
            textBox7.Text += ("Команда на проверку 'Микрофон' отправлена" + "\r\n");
            textBox7.Refresh();
            Thread.Sleep(250);
            test_end();
        }
        //private void test_mic_off ()
        //{
        //    ushort[] writeVals = new ushort[2];
        //    bool[] coilArr = new bool[4];
        //    startWrReg = 120;
        //    res = myProtocol.writeSingleRegister(slave, startWrReg, 7); // Отключить все сенсоры
        //    //startCoil = 38; // Запустить полный тест , адрес в контроллере 37
        //    //res = myProtocol.writeCoil(slave, startCoil, true);
        //    textBox7.Text += ("Команда на проверку отключения микрофона отправлена" + "\r\n");
        //    textBox7.Refresh();
        //    startCoil = 248;  //адрес сенсоров
        //    numCoils = 34;
        //    res = myProtocol.readInputDiscretes(slave, startCoil, coilArr, numCoils);
        //    if ((res == BusProtocolErrors.FTALK_SUCCESS))
        //    {
        //    }
        //    test_end();
        //}

        private void test_end()
        {
            ushort[] readVals = new ushort[21];
            bool[] coilArr = new bool[2];
            startRdReg = 120;                                    //regBank.add(40120);  // adr_control_command Адрес передачи комманд на выполнение
                                                                 //  0 в регистре означает завершение выполнения фрагмента проверки
            numRdRegs = 2;

            startCoil = 120;                                     // regBank.add(120);   // Флаг индикации возникновения любой ошибки
            numCoils = 2;
            Thread.Sleep(100);

            do
            {
                res = myProtocol.readMultipleRegisters(slave, 120, readVals, 1);  // Ожидание кода подтверждения окончания проверки  Адрес передачи подтверждения 40120

                if ((res == BusProtocolErrors.FTALK_SUCCESS))
                {
                    toolStripStatusLabel1.Text = "    MODBUS ON    ";
                    toolStripStatusLabel1.BackColor = Color.Lime;
                }

                else
                {
                    toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                    toolStripStatusLabel1.BackColor = Color.Red;
                    Polltimer1.Enabled = false;
                    timer_byte_set.Enabled = false;
                    timer_Mic_test.Enabled = false;
                    timerCTS.Enabled = false;
                    timerTestAll.Enabled = false;
                    return;
                }
                Thread.Sleep(50);
            } while (readVals[0] != 0);                                     // Если readVals[0] == 0 , тест завершен
            textBox8.Text += ("Тест закончен " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
            textBox8.Refresh();

            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);                       // Проверить Адрес 120  индикации возникновения любой ошибки
            if (coilArr[0] == true) //есть ошибка
            {
                // Обработка ошибки.
                textBox8.Text += ("Вызов программы обработки ошибок. " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture) + "\r\n");
                textBox8.Refresh();
                do{
                     error_list();
                   //  error_All();
                     res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);                       // Проверить Адрес 120  индикации возникновения любой ошибки
                     Thread.Sleep(50);
                   } while (coilArr[0] == true);    
              }

       }
        private void count_error_all()
        {
            ushort[] readVals = new ushort[2];
            startRdReg = 121;
            numRdRegs = 1;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40121  Считать данные общего счетчика ошибок  
            readVals[0]--;
            if (readVals[0] <= 0)
            {
                readVals[0] = 0;
                res = myProtocol.writeCoil(slave, 120, false);                                  // Снять флаг общей ошибки теста
            }
            startWrReg = 121;                                                                   // 
            res = myProtocol.writeSingleRegister(slave, startWrReg, readVals[0]);               // Команда на запись  счетчиков отправлена
            textBox8.Text += ("Общее количество ошибок  -  " + readVals[0] + "\r\n");
            textBox8.Refresh();
        }

        private void error_All()
        {


            ushort[] readVals = new ushort[12];
            ushort[] readVolt = new ushort[12];
            bool[] coilArr = new bool[12];
            startRdReg = 200;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40200 Считать счетчики ошибок  

            startCoil = 200;                                                                    // Начальный Адрес 200 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                textBox8.Text += ("Сенсор  трубки не отключился                                \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 200, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                textBox8.Text += ("Сенсор Тангента ручная не отключился                      \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 201, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                textBox8.Text += ("Сенсор Тангента ножная не отключился                      \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                textBox8.Refresh(); res = myProtocol.writeCoil(slave, 202, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора с 2 наушниками  не отключился\t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 203, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора  не отключился               \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 204, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                textBox8.Text += ("Сенсор диспетчера с 2 наушниками не отключился            \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 205, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                textBox8.Text += ("Сенсор диспетчера не отключился                               \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 206, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                textBox8.Text += ("Сенсор Микрофона не отключился                                \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 207, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                textBox8.Text += ("Микрофон инструктора не отключился                           \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 208, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                textBox8.Text += ("Микрофон диспетчера не отключился                            \t< = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 209, false);
                count_error_all();
            }

            startRdReg = 210;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40210 Считать счетчики ошибок  

            startCoil = 210;                                                                    // Начальный Адрес 210 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);

            if (coilArr[0] != false)
            {
                textBox8.Text += ("Сенсор  трубки не включился                                    \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 210, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                textBox8.Text += ("Сенсор Тангента ручная не включился                       \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 211, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                textBox8.Text += ("Сенсор Тангента ножная не включился                       \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 212, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора с 2 наушниками  не включился \t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 213, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора  не включился                \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 214, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                textBox8.Text += ("Сенсор диспетчера с 2 наушниками не включился          \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 215, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                textBox8.Text += ("Сенсор диспетчера не включился                               \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 216, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                textBox8.Text += ("Сенсор Микрофона не включился                                \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 217, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                textBox8.Text += ("Микрофон инструктора не включился                            \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 218, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                textBox8.Text += ("Микрофон диспетчера не включился                             \t< = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 219, false);
                count_error_all();
            }

            startRdReg = 220;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40220 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40420 Считать счетчики ошибок  
            startCoil = 220;                                                                    // Начальный Адрес 220 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                textBox8.Text += ("PTT инструктора не отключился                                \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 220, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                textBox8.Text += ("PTT инструктора не включился                                 \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 221, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                textBox8.Text += ("PTT диспетчера не отключился                                 \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 222, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                textBox8.Text += ("PTT диспетчера не включился                                 \t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 223, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Сигнал инструктора LineL отсутствует                      \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V \r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 224, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Сигнал инструктора LineR отсутствует                      \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V \r\n");
                res = myProtocol.writeCoil(slave, 225, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Сигнал инструктора на разъеме Маг phone отсутствует \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V \r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 226, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Сигнал диспетчера LineL отсутствует                       \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 227, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Сигнал диспетчера LineR отсутствует                       \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 228, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Сигнал диспетчера на разъеме Маг phone отсутствует    \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 229, false);
                count_error_all();
            }

            startRdReg = 230;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40230 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40430 Считать счетчики ошибок  
            startCoil = 230;                                                                      // Начальный Адрес 230 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test headset instructor ** Signal FrontL     \tOFF          \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 230, false);
                count_error_all();

            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test headset instructor ** Signal FrontR      \tOFF         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 231, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test headset instructor ** Signal LineL        \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 232, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test headset instructor ** Signal LineR        \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 233, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test headset instructor ** Signal mag radio    \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 234, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test headset instructor ** Signal mag phone    \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 235, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test headset instructor ** Signal GGS          \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 236, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test headset instructor ** Signal GG Radio1    \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 237, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test headset instructor ** Signal GG Radio2    \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 238, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("  < = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 239, false);
                count_error_all();
            }

            startRdReg = 240;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40240 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40440 Считать счетчики ошибок  
            startCoil = 240;                                                                      // Начальный Адрес 240 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test headset dispatcher ** Signal FrontL     \tOFF          \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 240, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test headset dispatcher ** Signal FrontR      \tOFF         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 241, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test headset dispatcher ** Signal LineL       \tOFF         \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 242, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test headset dispatcher ** Signal LineR       \tOFF         \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 243, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test headset dispatcher ** Signal mag radio    \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 244, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test headset dispatcher ** Signal mag phone    \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                res = myProtocol.writeCoil(slave, 245, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test headset dispatcher ** Signal GGS          \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 246, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test headset dispatcher ** Signal GG Radio1    \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 247, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test headset dispatcher ** Signal GG Radio2    \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 248, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("                                                          \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 249, false);
                count_error_all();
            }

            startRdReg = 250;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40250 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40450 Считать счетчики ошибок  
            startCoil = 250;                                                                      // Начальный Адрес 250 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test MTT ** Signal FrontL                      \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 250, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test MTT ** Signal FrontR                      \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 251, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test MTT ** Signal LineL                       \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 252, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test MTT ** Signal LineR                       \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 253, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test MTT ** Signal mag radio                   \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 254, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test MTT ** Signal mag phone                   \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 255, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test MTT ** Signal GGS                         \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 256, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test MTT ** Signal GG Radio1                   \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 257, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test MTT ** Signal GG Radio2                   \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 258, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test MTT ** Signal GGS                         \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 259, false);
                count_error_all();
            }

            startRdReg = 260;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40260 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40460 Считать напряжение ошибок  
            startCoil = 260;                                                                      // Начальный Адрес 260 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test MTT ** Signal LineL                       \tON         \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 260, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test MTT ** Signal LineR                       \tON         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 261, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test MTT ** Signal Mag phone                 \tON         \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 262, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test MTT PTT    (CTS)                          \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 263, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                textBox8.Text += ("Test microphone PTT  (CTS)                     \t OFF       \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 264, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                textBox8.Text += ("Test MTT PTT    (CTS)                          \tON         \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 265, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                textBox8.Text += ("Test microphone PTT  (CTS)                     \tON         \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 266, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                textBox8.Text += ("Test MTT HangUp (DCD)                          \tOFF        \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 267, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                textBox8.Text += ("Test MTT HangUp (DCD)                          \tON         \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 268, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                textBox8.Text += (" < = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 269, false);
                count_error_all();
            }

            startRdReg = 270;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40270 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40470 Считать напряжение ошибок  
            startCoil = 270;                                                                      // Начальный Адрес 270 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                textBox8.Text += ("Command PTT1 tangenta ruchnaja (CTS)           \tOFF        \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 270, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                textBox8.Text += ("Command PTT2 tangenta ruchnaja (DCR)           \tOFF        \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 271, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                textBox8.Text += ("Command PTT1 tangenta ruchnaja (CTS)           \tON         \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 272, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                textBox8.Text += ("Command PTT2 tangenta ruchnaja (DCR)           \tON         \t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 273, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                textBox8.Text += ("Command sensor tangenta ruchnaja               \tOFF        \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 274, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                textBox8.Text += ("Command sensor tangenta ruchnaja               \tON         \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 275, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                textBox8.Text += ("Command sensor tangenta nognaja                \tOFF        \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 276, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                textBox8.Text += ("Command sensor tangenta nognaja                \tON         \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 277, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                textBox8.Text += ("Command PTT tangenta nognaja (CTS)             \tOFF        \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 278, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                textBox8.Text += ("Command PTT tangenta nognaja (CTS)             \tON         \t< = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 279, false);
                count_error_all();
            }

            startRdReg = 280;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40280 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40480 Считать напряжение ошибок  
            startCoil = 280;                                                                      // Начальный Адрес 280 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test GGS ** Signal FrontL                      \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 280, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test GGS ** Signal FrontR                     \tOFF         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 281, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test GGS ** Signal LineL                       \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 282, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test GGS ** Signal LineR                       \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 283, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test GGS ** Signal mag radio                   \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 284, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test GGS ** Signal mag phone                   \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 285, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test GGS ** Signal GGS                         \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 286, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test GGS ** Signal GG Radio1                   \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 287, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test GGS ** Signal GG Radio2                   \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 288, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test GGS ** Signal GGS                         \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 289, false);
                count_error_all();
            }

            startRdReg = 290;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40290 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40490 Считать напряжение ошибок  
            startCoil = 290;                                                                      // Начальный Адрес 290 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test GGS ** Signal FrontL                      \tON         \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 290, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test GGS ** Signal FrontR                     \tON          \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 291, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test GGS ** Signal mag phone                   \tON         \t < = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 292, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                textBox8.Text += (" < = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 293, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                textBox8.Text += (" < = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 294, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                textBox8.Text += ("  < = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 295, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                textBox8.Text += ("  < = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 296, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                textBox8.Text += ("  < = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 297, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Microphone ** Signal mag phone            \tON         \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 298, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test Microphone ** Signal LineL                \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 299, false);
                count_error_all();
            }

            startRdReg = 300;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40300 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40500 Считать напряжение ошибок  
            startCoil = 300;                                                                      // Начальный Адрес 300 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test Radio1 ** Signal FrontL                   \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 300, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test Radio1 ** Signal FrontR                   \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 301, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test Radio1 ** Signal LineL                    \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 302, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test Radio1 ** Signal LineR                    \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 303, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test Radio1 ** Signal mag radio                \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 304, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test Radio1 ** Signal mag phone                \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 305, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test Radio1 ** Signal GGS                      \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 306, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test Radio1 ** Signal GG Radio1                \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 307, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Radio1 ** Signal GG Radio2                \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 308, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test Radio1 ** Signal Radio1                   \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 309, false);
                count_error_all();
            }

            startRdReg = 310;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40310 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40510 Считать напряжение ошибок  
            startCoil = 310;                                                                      // Начальный Адрес 310 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test Radio2 ** Signal FrontL                   \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 310, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test Radio2 ** Signal FrontR                   \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 311, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test Radio2 ** Signal LineL                    \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 312, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test Radio2 ** Signal LineR                    \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 313, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test Radio2 ** Signal mag radio                \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 314, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test Radio2 ** Signal mag phone                \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 315, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test Radio2 ** Signal GGS                      \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 316, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test Radio2 ** Signal GG Radio1                \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 317, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Radio2 ** Signal GG Radio2                \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 318, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test Radio2 ** Signal Radio2                   \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 319, false);
                count_error_all();
            }

            startRdReg = 320;
            numRdRegs = 10;
            res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40320 Считать счетчики ошибок  
            res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40520 Считать напряжение ошибок  
            startCoil = 320;                                                                      // Начальный Адрес 520 флага индикации возникновения  ошибки
            numCoils = 10;
            res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            Thread.Sleep(50);
            if (coilArr[0] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test Microphone ** Signal FrontL               \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 320, false);
                count_error_all();
            }

            if (coilArr[1] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test Microphone ** Signal FrontR               \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 321, false);
                count_error_all();
            }

            if (coilArr[2] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test Microphone ** Signal LineL                \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 322, false);
                count_error_all();
            }

            if (coilArr[3] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test Microphone ** Signal LineR                \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 323, false);
                count_error_all();
            }
            if (coilArr[4] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test Microphone ** Signal mag radio            \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 324, false);
                count_error_all();
            }
            if (coilArr[5] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test Microphone ** Signal mag phone            \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 325, false);
                count_error_all();
            }
            if (coilArr[6] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test Microphone ** Signal GGS                  \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 326, false);
                count_error_all();
            }
            if (coilArr[7] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test Microphone ** Signal GG Radio1            \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 327, false);
                count_error_all();
            }
            if (coilArr[8] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Microphone ** Signal GG Radio2            \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 328, false);
                count_error_all();
            }
            if (coilArr[9] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 329, false);
                count_error_all();
            }

            textBox8.Refresh();
            Thread.Sleep(50);
            res = myProtocol.readMultipleRegisters(slave, 120, readVals, 1);            // Ожидание кода подтверждения окончания проверки  Адрес передачи подтверждения 40120
            //if (readVals[0] == 0) ;                                                     // Если readVals[0] == 0 , тест завершен
            //    {
            //        res = myProtocol.writeCoil(slave, 120, false);                      // Снять флаг общей ошибки теста
            //    }
        }

        private void error_list()
        {
            ushort[] readVals = new ushort[10];
            ushort[] readVals_all = new ushort[200];
            ushort[] readVolt = new ushort[10];
            ushort[] readVolt_all = new ushort[200];
            bool[] coilArr = new bool[10];
            bool[] coilArr_all = new bool[200];
            richTextBox1.Text = "";
            startRdReg = 200;
            int startRdReg_temp = 0;
            numRdRegs = 5;
            startCoil = 200;                                                                    // Начальный Адрес 200 флага индикации возникновения  ошибки
            int startCoil_temp = 0; 
            numCoils = 5;

            //startRdReg = 220;
            //numRdRegs = 10;
            //res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40220 Считать счетчики ошибок  
            //res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40420 Считать счетчики ошибок  


            for (int i_reg = 0; i_reg < 13 ; i_reg++)
            {
                res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40200 Считать счетчики ошибок  
                res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs);
                res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
                for (int i_temp = 0; i_temp < 5; i_temp++)
                {
                    startRdReg_temp = startRdReg + i_temp;
                    startCoil_temp = startCoil + i_temp;
                    readVals_all[startRdReg_temp - 200] = readVals[i_temp];
                    readVolt_all[startRdReg_temp - 200] = readVolt[i_temp];
                    coilArr_all[startCoil_temp - 200] = coilArr[i_temp];
                   // richTextBox1.Text += (startRdReg + "  " + startRdReg_temp + "  " + startCoil_temp + "\r\n");
                    richTextBox1.Text += ((startRdReg_temp) + " - " + readVals_all[startRdReg_temp - 200] + " - " + coilArr_all[startCoil_temp - 200] + "\r\n");
                    richTextBox1.Refresh();
                   
                 }
                startRdReg += 5;
                startCoil += 5;
                res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);     // 40200 Считать счетчики ошибок  
                res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs);
                res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
                for (int i_temp = 0; i_temp < 10; i_temp++)
                {
                    startRdReg_temp = startRdReg + i_temp;
                    startCoil_temp = startCoil + i_temp;
                    readVals_all[startRdReg_temp - 200] = readVals[i_temp];
                    readVolt_all[startRdReg_temp - 200] = readVolt[i_temp];
                    coilArr_all[startCoil_temp - 200] = coilArr[i_temp];
                    richTextBox1.Text += ((startRdReg_temp) + " - " + readVals_all[startRdReg_temp - 200] + " - " + coilArr_all[startCoil_temp - 200] + "\r\n");
                    richTextBox1.Refresh();

                }
                startRdReg += 5;
                startCoil += 5;
            }
        
           
           if (coilArr_all[0] != false)
            {
                textBox8.Text += ("Сенсор  трубки не отключился                                \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 200, false);
                count_error_all();
            }

            if (coilArr_all[1] != false)
            {
                textBox8.Text += ("Сенсор Тангента ручная не отключился                      \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 201, false);
                count_error_all();
            }

            if (coilArr_all[2] != false)
            {
                textBox8.Text += ("Сенсор Тангента ножная не отключился                      \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                textBox8.Refresh(); res = myProtocol.writeCoil(slave, 202, false);
                count_error_all();
            }

            if (coilArr_all[3] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора с 2 наушниками  не отключился\t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 203, false);
                count_error_all();
            }
            if (coilArr_all[4] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора  не отключился               \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 204, false);
                count_error_all();
            }
            if (coilArr_all[5] != false)
            {
                textBox8.Text += ("Сенсор диспетчера с 2 наушниками не отключился            \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 205, false);
                count_error_all();
            }
            if (coilArr_all[6] != false)
            {
                textBox8.Text += ("Сенсор диспетчера не отключился                               \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 206, false);
                count_error_all();
            }
            if (coilArr_all[7] != false)
            {
                textBox8.Text += ("Сенсор Микрофона не отключился                                \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 207, false);
                count_error_all();
            }
            if (coilArr_all[8] != false)
            {
                textBox8.Text += ("Микрофон инструктора не отключился                           \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 208, false);
                count_error_all();
            }
            if (coilArr_all[9] != false)
            {
                textBox8.Text += ("Микрофон диспетчера не отключился                            \t< = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 209, false);
                count_error_all();
            }

            if (coilArr_all[10] != false)
            {
                textBox8.Text += ("Сенсор  трубки не включился                                    \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 210, false);
                count_error_all();
            }

            if (coilArr_all[11] != false)
            {
                textBox8.Text += ("Сенсор Тангента ручная не включился                       \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 211, false);
                count_error_all();
            }

            if (coilArr_all[12] != false)
            {
                textBox8.Text += ("Сенсор Тангента ножная не включился                       \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 212, false);
                count_error_all();
            }

            if (coilArr_all[13] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора с 2 наушниками  не включился \t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 213, false);
                count_error_all();
            }
            if (coilArr_all[14] != false)
            {
                textBox8.Text += ("Сенсор гарнитуры инструктора  не включился                \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 214, false);
                count_error_all();
            }
            if (coilArr_all[15] != false)
            {
                textBox8.Text += ("Сенсор диспетчера с 2 наушниками не включился          \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 215, false);
                count_error_all();
            }
            if (coilArr_all[16] != false)
            {
                textBox8.Text += ("Сенсор диспетчера не включился                               \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 216, false);
                count_error_all();
            }
            if (coilArr_all[17] != false)
            {
                textBox8.Text += ("Сенсор Микрофона не включился                                \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 217, false);
                count_error_all();
            }
            if (coilArr_all[18] != false)
            {
                textBox8.Text += ("Микрофон инструктора не включился                            \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 218, false);
                count_error_all();
            }
            if (coilArr_all[19] != false)
            {
                textBox8.Text += ("Микрофон диспетчера не включился                             \t< = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 219, false);
                count_error_all();
            }

            if (coilArr_all[20] != false)
            {
                textBox8.Text += ("PTT инструктора не отключился                                \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 220, false);
                count_error_all();
            }

            if (coilArr_all[21] != false)
            {
                textBox8.Text += ("PTT инструктора не включился                                 \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 221, false);
                count_error_all();
            }

            if (coilArr_all[22] != false)
            {
                textBox8.Text += ("PTT диспетчера не отключился                                 \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 222, false);
                count_error_all();
            }

            if (coilArr_all[23] != false)
            {
                textBox8.Text += ("PTT диспетчера не включился                                 \t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 223, false);
                count_error_all();
            }
            if (coilArr_all[24] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Сигнал инструктора LineL отсутствует                      \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V \r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 224, false);
                count_error_all();
            }
            if (coilArr_all[25] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Сигнал инструктора LineR отсутствует                      \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V \r\n");
                res = myProtocol.writeCoil(slave, 225, false);
                count_error_all();
            }
            if (coilArr_all[26] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Сигнал инструктора на разъеме Маг phone отсутствует \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V \r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 226, false);
                count_error_all();
            }
            if (coilArr_all[27] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Сигнал диспетчера LineL отсутствует                       \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 227, false);
                count_error_all();
            }
            if (coilArr_all[28] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Сигнал диспетчера LineR отсутствует                       \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 228, false);
                count_error_all();
            }
            if (coilArr_all[29] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Сигнал диспетчера на разъеме Маг phone отсутствует    \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 229, false);
                count_error_all();
            }

            if (coilArr_all[30] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test headset instructor ** Signal FrontL     \tOFF          \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 230, false);
                count_error_all();

            }

            if (coilArr_all[31] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test headset instructor ** Signal FrontR      \tOFF         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 231, false);
                count_error_all();
            }

            if (coilArr_all[32] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test headset instructor ** Signal LineL        \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 232, false);
                count_error_all();
            }

            if (coilArr_all[33] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test headset instructor ** Signal LineR        \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 233, false);
                count_error_all();
            }
            if (coilArr_all[34] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test headset instructor ** Signal mag radio    \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 234, false);
                count_error_all();
            }
            if (coilArr_all[35] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test headset instructor ** Signal mag phone    \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 235, false);
                count_error_all();
            }
            if (coilArr_all[36] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test headset instructor ** Signal GGS          \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 236, false);
                count_error_all();
            }
            if (coilArr_all[37] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test headset instructor ** Signal GG Radio1    \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 237, false);
                count_error_all();
            }
            if (coilArr_all[38] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test headset instructor ** Signal GG Radio2    \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 238, false);
                count_error_all();
            }
            if (coilArr_all[39] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("  < = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 239, false);
                count_error_all();
            }

            if (coilArr_all[40] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test headset dispatcher ** Signal FrontL     \tOFF          \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 240, false);
                count_error_all();
            }

            if (coilArr_all[41] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test headset dispatcher ** Signal FrontR      \tOFF         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 241, false);
                count_error_all();
            }

            if (coilArr_all[42] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test headset dispatcher ** Signal LineL       \tOFF         \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 242, false);
                count_error_all();
            }

            if (coilArr_all[43] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test headset dispatcher ** Signal LineR       \tOFF         \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 243, false);
                count_error_all();
            }
            if (coilArr_all[44] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test headset dispatcher ** Signal mag radio    \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 244, false);
                count_error_all();
            }
            if (coilArr_all[45] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test headset dispatcher ** Signal mag phone    \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                res = myProtocol.writeCoil(slave, 245, false);
                count_error_all();
            }
            if (coilArr_all[46] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test headset dispatcher ** Signal GGS          \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 246, false);
                count_error_all();
            }
            if (coilArr_all[47] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test headset dispatcher ** Signal GG Radio1    \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 247, false);
                count_error_all();
            }
            if (coilArr_all[48] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test headset dispatcher ** Signal GG Radio2    \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 248, false);
                count_error_all();
            }
            if (coilArr_all[49] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("                                                          \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 249, false);
                count_error_all();
            }

            if (coilArr_all[50] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test MTT ** Signal FrontL                      \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 250, false);
                count_error_all();
            }

            if (coilArr_all[51] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test MTT ** Signal FrontR                      \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 251, false);
                count_error_all();
            }

            if (coilArr_all[52] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test MTT ** Signal LineL                       \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 252, false);
                count_error_all();
            }

            if (coilArr_all[53] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test MTT ** Signal LineR                       \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 253, false);
                count_error_all();
            }
            if (coilArr_all[54] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test MTT ** Signal mag radio                   \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 254, false);
                count_error_all();
            }
            if (coilArr_all[55] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test MTT ** Signal mag phone                   \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 255, false);
                count_error_all();
            }
            if (coilArr_all[56] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test MTT ** Signal GGS                         \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 256, false);
                count_error_all();
            }
            if (coilArr_all[57] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test MTT ** Signal GG Radio1                   \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 257, false);
                count_error_all();
            }
            if (coilArr_all[58] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test MTT ** Signal GG Radio2                   \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 258, false);
                count_error_all();
            }
            if (coilArr_all[59] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test MTT ** Signal GGS                         \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 259, false);
                count_error_all();
            }

            if (coilArr_all[60] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test MTT ** Signal LineL                       \tON         \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 260, false);
                count_error_all();
            }

            if (coilArr_all[61] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test MTT ** Signal LineR                       \tON         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 261, false);
                count_error_all();
            }

            if (coilArr_all[62] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test MTT ** Signal Mag phone                 \tON         \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 262, false);
                count_error_all();
            }

            if (coilArr_all[63] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test MTT PTT    (CTS)                          \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 263, false);
                count_error_all();
            }
            if (coilArr_all[64] != false)
            {
                textBox8.Text += ("Test microphone PTT  (CTS)                     \t OFF       \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 264, false);
                count_error_all();
            }
            if (coilArr_all[65] != false)
            {
                textBox8.Text += ("Test MTT PTT    (CTS)                          \tON         \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 265, false);
                count_error_all();
            }
            if (coilArr_all[66] != false)
            {
                textBox8.Text += ("Test microphone PTT  (CTS)                     \tON         \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 266, false);
                count_error_all();
            }
            if (coilArr_all[67] != false)
            {
                textBox8.Text += ("Test MTT HangUp (DCD)                          \tOFF        \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 267, false);
                count_error_all();
            }
            if (coilArr_all[68] != false)
            {
                textBox8.Text += ("Test MTT HangUp (DCD)                          \tON         \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 268, false);
                count_error_all();
            }
            if (coilArr_all[69] != false)
            {
                textBox8.Text += (" < = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 269, false);
                count_error_all();
            }

            if (coilArr[70] != false)
            {
                textBox8.Text += ("Command PTT1 tangenta ruchnaja (CTS)           \tOFF        \t< = " + readVals[0] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 270, false);
                count_error_all();
            }

            if (coilArr_all[71] != false)
            {
                textBox8.Text += ("Command PTT2 tangenta ruchnaja (DCR)           \tOFF        \t< = " + readVals[1] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 271, false);
                count_error_all();
            }

            if (coilArr_all[72] != false)
            {
                textBox8.Text += ("Command PTT1 tangenta ruchnaja (CTS)           \tON         \t< = " + readVals[2] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 272, false);
                count_error_all();
            }

            if (coilArr_all[73] != false)
            {
                textBox8.Text += ("Command PTT2 tangenta ruchnaja (DCR)           \tON         \t< = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 273, false);
                count_error_all();
            }
            if (coilArr_all[74] != false)
            {
                textBox8.Text += ("Command sensor tangenta ruchnaja               \tOFF        \t< = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 274, false);
                count_error_all();
            }
            if (coilArr_all[75] != false)
            {
                textBox8.Text += ("Command sensor tangenta ruchnaja               \tON         \t< = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 275, false);
                count_error_all();
            }
            if (coilArr_all[76] != false)
            {
                textBox8.Text += ("Command sensor tangenta nognaja                \tOFF        \t< = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 276, false);
                count_error_all();
            }
            if (coilArr_all[77] != false)
            {
                textBox8.Text += ("Command sensor tangenta nognaja                \tON         \t< = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 277, false);
                count_error_all();
            }
            if (coilArr_all[78] != false)
            {
                textBox8.Text += ("Command PTT tangenta nognaja (CTS)             \tOFF        \t< = " + readVals[8] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 278, false);
                count_error_all();
            }
            if (coilArr_all[79] != false)
            {
                textBox8.Text += ("Command PTT tangenta nognaja (CTS)             \tON         \t< = " + readVals[9] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 279, false);
                count_error_all();
            }

            if (coilArr_all[80] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test GGS ** Signal FrontL                      \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 280, false);
                count_error_all();
            }

            if (coilArr_all[81] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test GGS ** Signal FrontR                     \tOFF         \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 281, false);
                count_error_all();
            }

            if (coilArr_all[82] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test GGS ** Signal LineL                       \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 282, false);
                count_error_all();
            }

            if (coilArr_all[83] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test GGS ** Signal LineR                       \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 283, false);
                count_error_all();
            }
            if (coilArr_all[84] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test GGS ** Signal mag radio                   \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 284, false);
                count_error_all();
            }
            if (coilArr_all[85] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test GGS ** Signal mag phone                   \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 285, false);
                count_error_all();
            }
            if (coilArr_all[86] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test GGS ** Signal GGS                         \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 286, false);
                count_error_all();
            }
            if (coilArr_all[87] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test GGS ** Signal GG Radio1                   \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 287, false);
                count_error_all();
            }
            if (coilArr_all[88] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test GGS ** Signal GG Radio2                   \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 288, false);
                count_error_all();
            }
            if (coilArr_all[89] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test GGS ** Signal GGS                         \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 289, false);
                count_error_all();
            }

            if (coilArr_all[90] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test GGS ** Signal FrontL                      \tON         \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 290, false);
                count_error_all();
            }

            if (coilArr_all[91] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test GGS ** Signal FrontR                     \tON          \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 291, false);
                count_error_all();
            }

            if (coilArr_all[92] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test GGS ** Signal mag phone                   \tON         \t < = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 292, false);
                count_error_all();
            }

            if (coilArr_all[93] != false)
            {
                textBox8.Text += (" < = " + readVals[3] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 293, false);
                count_error_all();
            }
            if (coilArr_all[94] != false)
            {
                textBox8.Text += (" < = " + readVals[4] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 294, false);
                count_error_all();
            }
            if (coilArr_all[95] != false)
            {
                textBox8.Text += ("  < = " + readVals[5] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 295, false);
                count_error_all();
            }
            if (coilArr_all[96] != false)
            {
                textBox8.Text += ("  < = " + readVals[6] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 296, false);
                count_error_all();
            }
            if (coilArr_all[97] != false)
            {
                textBox8.Text += ("  < = " + readVals[7] + ">\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 297, false);
                count_error_all();
            }
            if (coilArr_all[98] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Microphone ** Signal mag phone            \tON         \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 298, false);
                count_error_all();
            }
            if (coilArr_all[99] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test Microphone ** Signal LineL                \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 299, false);
                count_error_all();
            }

            if (coilArr_all[100] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test Radio1 ** Signal FrontL                   \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 300, false);
                count_error_all();
            }

            if (coilArr_all[101] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test Radio1 ** Signal FrontR                   \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 301, false);
                count_error_all();
            }

            if (coilArr_all[102] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test Radio1 ** Signal LineL                    \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 302, false);
                count_error_all();
            }

            if (coilArr_all[103] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test Radio1 ** Signal LineR                    \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 303, false);
                count_error_all();
            }
            if (coilArr_all[104] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test Radio1 ** Signal mag radio                \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 304, false);
                count_error_all();
            }
            if (coilArr_all[105] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test Radio1 ** Signal mag phone                \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 305, false);
                count_error_all();
            }
            if (coilArr_all[106] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test Radio1 ** Signal GGS                      \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 306, false);
                count_error_all();
            }
            if (coilArr_all[107] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test Radio1 ** Signal GG Radio1                \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 307, false);
                count_error_all();
            }
            if (coilArr_all[108] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Radio1 ** Signal GG Radio2                \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 308, false);
                count_error_all();
            }
            if (coilArr_all[109] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test Radio1 ** Signal Radio1                   \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 309, false);
                count_error_all();
            }
             
            if (coilArr_all[110] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test Radio2 ** Signal FrontL                   \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 310, false);
                count_error_all();
            }

            if (coilArr_all[111] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test Radio2 ** Signal FrontR                   \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 311, false);
                count_error_all();
            }

            if (coilArr_all[112] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test Radio2 ** Signal LineL                    \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 312, false);
                count_error_all();
            }

            if (coilArr_all[113] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test Radio2 ** Signal LineR                    \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 313, false);
                count_error_all();
            }
            if (coilArr_all[114] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test Radio2 ** Signal mag radio                \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 314, false);
                count_error_all();
            }
            if (coilArr_all[115] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test Radio2 ** Signal mag phone                \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 315, false);
                count_error_all();
            }
            if (coilArr_all[116] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test Radio2 ** Signal GGS                      \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 316, false);
                count_error_all();
            }
            if (coilArr_all[117] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test Radio2 ** Signal GG Radio1                \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 317, false);
                count_error_all();
            }
            if (coilArr_all[118] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Radio2 ** Signal GG Radio2                \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 318, false);
                count_error_all();
            }
            if (coilArr_all[119] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("Test Radio2 ** Signal Radio2                   \tON         \t< = " + readVals[9] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 319, false);
                count_error_all();
            }

            //startRdReg = 320;
            //numRdRegs = 10;
            //res = myProtocol.readMultipleRegisters(slave, startRdReg, readVals, numRdRegs);       // 40320 Считать счетчики ошибок  
            //res = myProtocol.readMultipleRegisters(slave, startRdReg + 200, readVolt, numRdRegs); // 40520 Считать напряжение ошибок  
            //startCoil = 320;                                                                      // Начальный Адрес 520 флага индикации возникновения  ошибки
            //numCoils = 10;
            //res = myProtocol.readCoils(slave, startCoil, coilArr, numCoils);
            //Thread.Sleep(50);
            if (coilArr_all[120] != false)
            {
                temp_disp = readVolt[0];
                textBox8.Text += ("Test Microphone ** Signal FrontL               \tOFF        \t< = " + readVals[0] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 320, false);
                count_error_all();
            }

            if (coilArr_all[121] != false)
            {
                temp_disp = readVolt[1];
                textBox8.Text += ("Test Microphone ** Signal FrontR               \tOFF        \t< = " + readVals[1] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 321, false);
                count_error_all();
            }

            if (coilArr_all[122] != false)
            {
                temp_disp = readVolt[2];
                textBox8.Text += ("Test Microphone ** Signal LineL                \tOFF        \t< = " + readVals[2] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 322, false);
                count_error_all();
            }

            if (coilArr_all[123] != false)
            {
                temp_disp = readVolt[3];
                textBox8.Text += ("Test Microphone ** Signal LineR                \tOFF        \t< = " + readVals[3] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 323, false);
                count_error_all();
            }
            if (coilArr_all[124] != false)
            {
                temp_disp = readVolt[4];
                textBox8.Text += ("Test Microphone ** Signal mag radio            \tOFF        \t< = " + readVals[4] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 324, false);
                count_error_all();
            }
            if (coilArr_all[125] != false)
            {
                temp_disp = readVolt[5];
                textBox8.Text += ("Test Microphone ** Signal mag phone            \tOFF        \t< = " + readVals[5] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 325, false);
                count_error_all();
            }
            if (coilArr_all[126] != false)
            {
                temp_disp = readVolt[6];
                textBox8.Text += ("Test Microphone ** Signal GGS                  \tOFF        \t< = " + readVals[6] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 326, false);
                count_error_all();
            }
            if (coilArr_all[127] != false)
            {
                temp_disp = readVolt[7];
                textBox8.Text += ("Test Microphone ** Signal GG Radio1            \tOFF        \t< = " + readVals[7] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 327, false);
                count_error_all();
            }
            if (coilArr_all[128] != false)
            {
                temp_disp = readVolt[8];
                textBox8.Text += ("Test Microphone ** Signal GG Radio2            \tOFF        \t< = " + readVals[8] + ">  " + temp_disp / 100 + " V\r\n");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 328, false);
                count_error_all();
            }
            if (coilArr_all[129] != false)
            {
                temp_disp = readVolt[9];
                textBox8.Text += ("");
                textBox8.Refresh();
                res = myProtocol.writeCoil(slave, 329, false);
                count_error_all();
            }
        }

        //*******************************************

        private void timerTestAll_Tick(object sender, EventArgs e)        // Тестирование программы общего теста
        {

            switch (TestN)  // Определить № теста
            {
                default:
                case 0:
                    if (checkBoxSensors1.Checked || radioButton1.Checked)
                    {
                        sensor_off();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;

                case 1:
                    if (checkBoxSensors2.Checked || radioButton1.Checked)
                    {
                        sensor_on();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 2:
                    if (checkBoxSenGar1instr.Checked || radioButton1.Checked)
                    {
                        test_instruktora();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 3:
                    if (checkBoxSenGar1disp.Checked || radioButton1.Checked)
                    {
                        test_dispetchera();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 4:
                    if (checkBoxSenTrubka.Checked || radioButton1.Checked)
                    {
                        test_MTT();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 5:
                    if (checkBoxSenTangRuch.Checked || radioButton1.Checked)
                    {
                        test_tangR();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 6:
                    if (checkBoxSenTangN.Checked || radioButton1.Checked)
                    {
                        test_tangN();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 7:
                    if (checkBoxSenGGS.Checked || radioButton1.Checked)
                    {
                        testGGS();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 8:
                    if (checkBoxSenGGRadio1.Checked || radioButton1.Checked)
                    {
                        test_GG_Radio1();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 9:
                    if (checkBoxSenGGRadio2.Checked || radioButton1.Checked)
                    {
                        test_GG_Radio2();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;
                case 10:
                    if (checkBoxSenMicrophon.Checked || radioButton1.Checked)
                    {
                        test_mikrophon();
                        progressBar2.Value += 1;
                        label98.Text = ("" + progressBar2.Value);
                        label98.Refresh();
                    }
                    break;

            }

            TestN++;                  // Увеличить счетчик проходов теста

            label98.Text = ("" + progressBar2.Value);
            if (progressBar2.Value == progressBar2.Maximum)
            {
                progressBar2.Value = 0;
            }
            label80.Text = DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture);
            toolStripStatusLabel2.Text = ("Время : " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture));

            if (radioButton1.Checked & TestN == 11)
            {
                timerTestAll.Enabled = false;
                textBox7.Text += ("Тест завершен" + "\r\n");
                textBox7.Text += ("\r\n");

                _All_Test_Stop = false;
            }
            if (radioButton2.Checked & TestN == 11)
            {
                timerTestAll.Enabled = true;
                TestN = 0;
                TestRepeatCount++;
                if (TestRepeatCount > 32767) TestRepeatCount = 1;
                textBox7.Text += ("\r\n");
                textBox7.Text += ("Повтор теста " + TestRepeatCount + "\r\n");
                textBox7.Text += ("\r\n");
                Thread.Sleep(1500);
            }
        }

        private void button11_Click_1(object sender, EventArgs e)         //Старт полного теста
        {
            Polltimer1.Enabled = false;
            timer_Mic_test.Enabled = false;
            timer_byte_set.Enabled = false;
            ushort[] writeVals = new ushort[200];
            bool[] coilArr = new bool[200];
            progressBar2.Value = 0;
            startWrReg = 121;                                                                   // 
            res = myProtocol.writeSingleRegister(slave, startWrReg, 0);                         // Команда на обнуление счетчика общих ошибок отправлена
            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
            button11.BackColor = Color.Lime;
            button11.Refresh();
            label92.Text = ("Не забываем нажать кнопку [СТОП] после выполнения теста!");
            button9.BackColor = Color.LightSalmon;
            button9.Refresh();
            textBox7.Text = ("Выполняется полный  контроль звукового модуля Камертон " + "\r\n");
            textBox7.Text += ("\r\n");
            textBox8.Text = ("");
            textBox9.Text = ("");
            textBox7.Refresh();
            textBox8.Refresh();
            textBox9.Refresh();
            if (radioButton2.Checked)
            {
                startCoil = 118;                                                          // Управление сенсорами
                res = myProtocol.writeCoil(slave, startCoil, true);
            }
            else
            {
                startCoil = 118;                                                          // Управление сенсорами
                res = myProtocol.writeCoil(slave, startCoil, false);
            }


            TestN = 0;                                                                                  // Обнулить счетчик количества выполняемых тестов
            TestRepeatCount = 1;                                                                        // Установить начальный номер  счетчика проходов теста

            if (_All_Test_Stop)                                                                         // Проверить наличие завершения выполнения тестов
            {

                /*
                     // textBox9.Text += ("Старт Обнуление счетчиков  - " + _All_Test_Stop + "\r\n");
                    for (int err = 0; err < 100; err++)                                                 //  Обнулить регистры счетчиков ошибок
                        {
                            writeVals[err] = 0;
                        }

                    //startWrReg = 121;                                                                   //  Обнулить счетчики ошибок
                    //numWrRegs = 14;
                    //res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);   // Выполнить сброс счетчиков 40121 - 40130

                    startWrReg = 200;                                                                   //  Обнулить счетчики ошибок
                    numWrRegs = 10;
                    res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);   // Выполнить сброс счетчиков 40200 - 40210
                    startWrReg = 210;                                                                   //  Обнулить счетчики ошибок
                    numWrRegs = 10;
                    res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);   // Выполнить сброс счетчиков 40131 - 40140
                    startWrReg = 220;                                                                   //  Обнулить счетчики ошибок
                    numWrRegs = 10;
                    res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);   // Выполнить сброс счетчиков 40141 - 40150
                    // Thread.Sleep(600);
              
                    for (int err = 0; err < 100; err++)                                                 //  Обнулить регистры счетчиков ошибок
                    {
                        coilArr[err] = false;
                    }
                    startCoil = 200;
                    numCoils = 10;
                    res = myProtocol.forceMultipleCoils(slave, startCoil, coilArr, numCoils);          // 15 (0F) Записать бит false или true  по адресу 0-9999 
                    startCoil = 210;
                    numCoils = 10;
                    res = myProtocol.forceMultipleCoils(slave, startCoil, coilArr, numCoils);          // 15 (0F) Записать бит false или true  по адресу 0-9999 
                    startCoil = 220;
                    numCoils = 10;
                    res = myProtocol.forceMultipleCoils(slave, startCoil, coilArr, numCoils);          // 15 (0F) Записать бит false или true  по адресу 0-9999                 
                  
                */

                startWrReg = 120;                                                                   // Команда на 
                res = myProtocol.writeSingleRegister(slave, startWrReg, 16);                        // Команда на сброс счетчиков отправлена
                Thread.Sleep(500);
                startWrReg = 120;                                                                   // Команда на открытие файла отправлена
                res = myProtocol.writeSingleRegister(slave, startWrReg, 12);                        // Команда на открытие файла отправлена
                textBox9.Text += ("Команда на открытие файла отправлена" + "\r\n");
                textBox7.Refresh();
                file_fakt_namber();
                Thread.Sleep(500);
                _All_Test_Stop = false;                                                             // Установить флаг запуска теста
            }
            /*
                        do
                        {
                            textBox7.Text += ("\r\n");
                            textBox7.Text += ("Выполнение теста N  " + TestRepeatCount + "\r\n");
                            textBox7.Text += ("\r\n");
                
                               button9.Refresh();
                              //  Thread.Sleep(500);
                              //  sensor_off();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                              //  Thread.Sleep(500);
                              //  sensor_on();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                            //    Thread.Sleep(500);
                                test_instruktora();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                          //      Thread.Sleep(500);
                                test_dispetchera();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                           //     Thread.Sleep(500);
                                test_MTT();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                             //   Thread.Sleep(500);
                                test_tangR();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                            //    Thread.Sleep(500);
                                test_mikrophon();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                            //    Thread.Sleep(500);
                                testGGS();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                             //   Thread.Sleep(500);
                                test_GG_Radio1();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                         //       Thread.Sleep(500);
                                test_GG_Radio2();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                           //     Thread.Sleep(500);
                                test_mag();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                          //      Thread.Sleep(500);
                                test_mic_off();
                                progressBar2.Value += 1;
                                if (progressBar2.Value == 100) progressBar2.Value = 0;
                                label98.Text = ("" + progressBar2.Value);
                                label98.Refresh();

                                button9.Refresh();
                           //     Thread.Sleep(500);
                                TestRepeatCount++;
                                if (TestRepeatCount > 32767) TestRepeatCount = 1;
                                Thread.Sleep(100);

                            //if (_All_Test_Stop == true)
                            //{
                            //    break;
                            //}

                        } while (radioButton2.Checked && _All_Test_Stop == false);


                        */
            //-------------------------------------------------
            timerTestAll.Enabled = true;


            if ((res == BusProtocolErrors.FTALK_SUCCESS))
            {
                toolStripStatusLabel1.Text = "    MODBUS ON    ";
                toolStripStatusLabel1.BackColor = Color.Lime;
            }
            else
            {
                toolStripStatusLabel1.Text = "    MODBUS ERROR   ";
                toolStripStatusLabel1.BackColor = Color.Red;
                // Polltimer1.Enabled = false;
                timer_Mic_test.Enabled = false;
            }

            progressBar2.Value = 0;

            label80.Text = DateTime.Now.ToString("dd.MM.yyyy HH:mm:ss", CultureInfo.CurrentCulture);
            toolStripStatusLabel2.Text = ("Время : " + DateTime.Now.ToString("dd/MM/yyyy HH:mm:ss", CultureInfo.CurrentCulture));
        }

        private void button9_Click(object sender, EventArgs e)            // Стоп полного теста
        {
            timerTestAll.Enabled = false;

            ushort[] writeVals = new ushort[20];
            bool[] coilArr = new bool[34];

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);
            button9.BackColor = Color.Red;
            button11.BackColor = Color.White;
            label92.Text = ("");
            textBox7.Text += ("Тест остановлен" + "\r\n");
            progressBar2.Value = 0;
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 13); // Команда на закрытие файла отправлена
            textBox9.Text += ("Команда на закрытие файла отправлена" + "\r\n");
            // textBox9.Text = ("Стоп теста");
            textBox7.Refresh();
            textBox9.Refresh();
            //    test_end();

            textBox7.Text += "Тест окончен!";

            _All_Test_Stop = true;
            Polltimer1.Enabled = true;
        }

        private void label92_Click(object sender, EventArgs e)
        {

        }

        private void button12_Click(object sender, EventArgs e)          // Создать файл
        {
            if (!File.Exists(fileName))
            {
                File.Create(fileName).Close();
            }
            else
            {
                MessageBox.Show("Файл уже существует!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }

        }

        private void button13_Click(object sender, EventArgs e)          // Удалить файл 
        {
            if (File.Exists(fileName))
            {
                File.Delete(fileName);
            }
            else
            {
                //MessageBox.Show("Файл НЕ существует!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Asterisk);
                MessageBox.Show("Файл НЕ существует!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }

        }

        private void button21_Click(object sender, EventArgs e)          // Записать текст в файл 
        {

            File.WriteAllText(fileName, textBox9.Text);
        }

        private void button15_Click(object sender, EventArgs e)          // Прочитать файл
        {
            if (File.Exists(fileName))
            {
                textBox9.Text = File.ReadAllText(fileName);
            }
            else
            {
                MessageBox.Show("Файл НЕ существует!", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }

        }

        private void label145_Click(object sender, EventArgs e)
        {

        }

        private void label148_Click(object sender, EventArgs e)
        {

        }

        private void toolStripStatusLabel3_Click(object sender, EventArgs e)
        {

        }

        private void label78_Click(object sender, EventArgs e)
        {

        }

        private void lblResult_Click(object sender, EventArgs e)
        {

        }

        private void progressBar2_Click(object sender, EventArgs e)
        {

        }

        private void statusStrip1_ItemClicked(object sender, ToolStripItemClickedEventArgs e)
        {

        }

        private void label128_Click(object sender, EventArgs e)
        {

        }

        private void textBox4_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox7_TextChanged(object sender, EventArgs e)
        {
            textBox7.SelectionStart = textBox7.Text.Length;
            textBox7.ScrollToCaret();
            textBox7.Refresh();
        }

        private void textBox8_TextChanged(object sender, EventArgs e)
        {
            textBox8.SelectionStart = textBox8.Text.Length;
            textBox8.ScrollToCaret();
            textBox8.Refresh();
        }

        private void textBox9_TextChanged(object sender, EventArgs e)
        {
            textBox9.SelectionStart = textBox9.Text.Length;
            textBox9.ScrollToCaret();
            textBox9.Refresh();
        }

        private void groupBox21_Enter(object sender, EventArgs e)
        {

        }
        #endregion


        private void textBox11_TextChanged(object sender, EventArgs e)
        {
            textBox11.SelectionStart = textBox11.Text.Length;
            textBox11.ScrollToCaret();
            textBox11.Refresh();
        }

        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {

        }


        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBoxSenAll.Checked == true)
            {
                // Включить проверку всех сенсоров
                checkBoxSensors1.Checked = true;
                checkBoxSensors2.Checked = true;
                checkBoxSenGar1instr.Checked = true;
                checkBoxSenGar1disp.Checked = true;
                checkBoxSenTrubka.Checked = true;
                checkBoxSenTangN.Checked = true;
                checkBoxSenTangRuch.Checked = true;
                checkBoxSenMicrophon.Checked = true;
                checkBoxSenGGRadio1.Checked = true;
                checkBoxSenGGRadio2.Checked = true;
                checkBoxSenGGS.Checked = true;
            }
            else
            {
                // Отключить проверку всех сенсоров
                checkBoxSenGGRadio1.Checked = false;
                checkBoxSenGGRadio2.Checked = false;
                checkBoxSenTrubka.Checked = false;
                checkBoxSenTangN.Checked = false;
                checkBoxSenTangRuch.Checked = false;
                checkBoxSensors1.Checked = false;
                checkBoxSensors2.Checked = false;
                checkBoxSenGar1instr.Checked = false;
                checkBoxSenGar1disp.Checked = false;
                checkBoxSenMicrophon.Checked = false;
                checkBoxSenGGS.Checked = false;
            }

        }

        private void checkBoxSenGGRadio1_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void label30_Click(object sender, EventArgs e)
        {

        }

        private void label33_Click(object sender, EventArgs e)
        {

        }

        private void label83_Click(object sender, EventArgs e)
        {

        }

        private void label71_Click(object sender, EventArgs e)
        {

        }

        private void textBox3_TextChanged(object sender, EventArgs e)
        {

        }

        private void button24_Click(object sender, EventArgs e)
        {
            //Polltimer1.Enabled = false;
            //timer_byte_set.Enabled = false;
            //timerCTS.Enabled = false;
            //timerTestAll.Enabled = false;
            short[] writeVals = new short[12];
            short[] MSK = new short[2];
            MSK[0] = 5;
            ushort[] readVals = new ushort[125];

            bool[] coilVals = new bool[200];
            bool[] coilArr = new bool[20];

            slave = int.Parse(txtSlave.Text, CultureInfo.CurrentCulture);

            textBox4.BackColor = Color.White;
            writeVals[1] = short.Parse(textBox4.Text, CultureInfo.CurrentCulture);   // Установка уровня входного сигнала
            int tempK = writeVals[1] * 5;                                            // Установка уровня входного сигнала
            if (tempK > 250)
            {
                label72.Text = "<";
                tempK = 250;
                textBox4.Text = "50";
                textBox4.BackColor = Color.Red;
            }
            else
            {
                label72.Text = "=";
                textBox4.BackColor = Color.White;
            }
            //writeVals[1] = (short) tempK;                 // Установка уровня входного сигнала
            //startWrReg = 41;
            //numWrRegs = 10;                               //
            //res = myProtocol.writeMultipleRegisters(slave, startWrReg, writeVals, numWrRegs);
            startWrReg = 10;                                                                   // 
            res = myProtocol.writeSingleRegister(slave, startWrReg, (short)tempK);
            startWrReg = 120;                                                                   // 
            res = myProtocol.writeSingleRegister(slave, startWrReg, 15);                        // 

        }

        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void checkBoxSenGar2instr_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void label92_Click_2(object sender, EventArgs e)
        {

        }

        private void label95_Click(object sender, EventArgs e)
        {

        }

        private void button81_Click(object sender, EventArgs e)            //  Установить номер файла в '0'
        {
            ushort[] writeVals = new ushort[2];
            bool[] coilArr = new bool[4];
            startWrReg = 120;
            res = myProtocol.writeSingleRegister(slave, startWrReg, 17);   // 
            // test_end();
        }

        private void label133_Click_1(object sender, EventArgs e)
        {

        }

        private void label80_Click(object sender, EventArgs e)
        {

        }

        private void label37_Click(object sender, EventArgs e)
        {

        }

        private void checkBoxSensors1_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            // Получить список ошибок

            error_list();

        }


    }
}

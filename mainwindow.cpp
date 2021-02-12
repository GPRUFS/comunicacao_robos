#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Teste de Comunicação");
    serial_config();

    graph_config();

    ui->label_2->setNum(0);
    ui->label_4->setNum(0);

    ui->pushStartStop->setText("Iniciar");
    ui->verticalSlider_1_L->setRange(-100,100);
    ui->verticalSlider_1_R->setRange(-100,100);
    ui->verticalSlider_2_L->setRange(-100,100);
    ui->verticalSlider_2_R->setRange(-100,100);
    ui->verticalSlider_3_L->setRange(-100,100);
    ui->verticalSlider_3_R->setRange(-100,100);
    ui->verticalSlider_4_L->setRange(-100,100);
    ui->verticalSlider_4_R->setRange(-100,100);
    ui->verticalSlider_5_L->setRange(-100,100);
    ui->verticalSlider_5_R->setRange(-100,100);

    ui->spinBox_1_L->setRange(-100,100);
    ui->spinBox_1_R->setRange(-100,100);
    ui->spinBox_2_L->setRange(-100,100);
    ui->spinBox_2_R->setRange(-100,100);
    ui->spinBox_3_L->setRange(-100,100);
    ui->spinBox_3_R->setRange(-100,100);
    ui->spinBox_4_L->setRange(-100,100);
    ui->spinBox_4_R->setRange(-100,100);
    ui->spinBox_5_L->setRange(-100,100);
    ui->spinBox_5_R->setRange(-100,100);

    write_buf[0] = (unsigned char) (250); //Inicializando caso eu não queira monitorar nenhum

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout,
            this, &MainWindow::enviar_buffer);
    timer->start(10);

}

MainWindow::~MainWindow()
{
    delete ui;
}

/* CONVERSÃO DOS DADOS ENVIADOS OU RECEBIDOS  */
unsigned char MainWindow::converter_write (int x)
{
    //OBSERVAÇÃO: SÓ FUNCIONA PRA VALORES MENORES QUE 127, MAS O PROTOCOLO É DE 0 A 100 COM BIT DE SINAL

    unsigned char c = C0; //Inicializando com zero
    if (x >= 0)
    {
        c = (unsigned char) x;
    }
    else
    {
        c = (unsigned char) (abs(x)+128); //Módulo + o bit de sinal
    }
    return c;
}
int MainWindow::converter_read (unsigned char c)
{
    int x = 0; //Inicializando com zero
    if (((int)c) >= 128)
        x = -(((int)c) - 128);
    else
        x = (int)c;
    return x;
}

/* INICIAR OU ENCERRAR A COMUNICAÇÃO */

void MainWindow::on_pushStartStop_clicked()
{
    flag_comunicacao = !flag_comunicacao;
    if (flag_comunicacao)
    {
        ui->pushStartStop->setText("Parar");
    }
    else
    {
        ui->pushStartStop->setText("Iniciar");
    }
}

void MainWindow::enviar_buffer(void)
{
    if (flag_comunicacao)
    {
        /*QString string_envio;
        string_envio.append(QString::number(write_buf[0]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[1]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[2]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[3]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[4]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[5]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[6]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[7]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[8]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[9]));
        string_envio.append(" - ");
        string_envio.append(QString::number(write_buf[10]));
        ui->label_envio->setText(string_envio);*/


        write(serial_port, write_buf, sizeof(write_buf));
        usleep(10); //10us
        ler_buffer();
    }
}

void MainWindow::ler_buffer(void)
{
    if (flag_robos!=0)
    {
        read(serial_port, &read_buf, sizeof(read_buf)); //Só lê se tiver de quem ler -- talvez dê ruim
        //Atualização do plot
        yplot1.pop_front(); //Removi o primeiro elemento
        yplot1.push_back(converter_read(read_buf[L1])); //Adicionei um novo elemento no final
        yplot2.pop_front(); //Removi o primeiro elemento
        yplot2.push_back(converter_read(read_buf[R1])); //Adicionei um novo elemento no final
        ui->widget->graph(0)->setData(xplot, yplot1);
        ui->widget->graph(1)->setData(xplot, yplot2);
        ui->widget->replot();
        //Cálculo da média
        ui->label_2->setNum((int)std::accumulate(yplot1.begin(), yplot1.end(), .0) / yplot1.size());
        ui->label_4->setNum((int)std::accumulate(yplot2.begin(), yplot2.end(), .0) / yplot2.size());

        QString string_envio;
        string_envio.append(QString::number(read_buf[0]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[1]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[2]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[3]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[4]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[5]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[6]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[7]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[8]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[9]));
        string_envio.append(" - ");
        string_envio.append(QString::number(read_buf[10]));
        ui->label_envio->setText(string_envio);
    }
}

/* ESCOLHER QUAL ROBÔ MONITORAR */
void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    flag_robos = index;
    write_buf[0] = (unsigned char) (250+index);
    /*switch (flag_robos) {
        case 0:
            write_buf[0] = C0;
        default: //TODO: Incluir outros casos aqui
            write_buf[0] = (unsigned char) 255;
    }*/
}

/* CONFIGURAÇÃO DA PORTA SERIAL */

void MainWindow::serial_config(void)
{
    serial_port = open("/dev/ttyACM0", O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    // Allocate memory for read buffer, set size according to your needs
    memset(&read_buf, '\0', sizeof(read_buf));
}

/* CONFIGURAÇÃO DO GRÁFICO */

void MainWindow::graph_config(void)
{
    QVector<double> x(101), y(101); // initialize with entries 0..100

    xplot = x;
    yplot1 = y;
    yplot2 = y;

    for (int i=0; i<101; ++i)
    {
      xplot[i] = i; // Inicializando
      yplot1[i] = 0;
      yplot2[i] = 0;
    }
    // create graph and assign data to it:
    ui->widget->addGraph();

    ui->widget->addGraph();
    //ui->widget->graph(1)->setData(x, x);
    ui->widget->graph(1)->setPen(QPen(Qt::red));
    // give the axes some labels:
    ui->widget->yAxis->setLabel("Vel");
    // set axes ranges, so we see all data:
    ui->widget->xAxis->setRange(0, 100);
    ui->widget->yAxis->setRange(-105, 105);
    //tirando o label inferior
    ui->widget->xAxis->setVisible(false);
    ui->widget->replot();
}

/* SELEÇÃO DE VELOCIDADES */

//Robô 1 - L
void MainWindow::on_verticalSlider_1_L_valueChanged(int value)
{
    ui->spinBox_1_L->setValue(value);
    write_buf[L1] = converter_write(value);
}
void MainWindow::on_spinBox_1_L_valueChanged(int arg1)
{
    ui->verticalSlider_1_L->setValue(arg1);
    write_buf[L1] = converter_write(arg1);
}
//Robô 1 - R
void MainWindow::on_verticalSlider_1_R_valueChanged(int value)
{
    ui->spinBox_1_R->setValue(value);
    write_buf[R1] = converter_write(value);
}
void MainWindow::on_spinBox_1_R_valueChanged(int arg1)
{
    ui->verticalSlider_1_R->setValue(arg1);
    write_buf[R1] = converter_write(arg1);
}
//Robô 2 - L
void MainWindow::on_verticalSlider_2_L_valueChanged(int value)
{
    ui->spinBox_2_L->setValue(value);
    write_buf[L2] = converter_write(value);
}
void MainWindow::on_spinBox_2_L_valueChanged(int arg1)
{
    ui->verticalSlider_2_L->setValue(arg1);
    write_buf[L2] = converter_write(arg1);
}
//Robô 2 - R
void MainWindow::on_verticalSlider_2_R_valueChanged(int value)
{
    ui->spinBox_2_R->setValue(value);
    write_buf[R2] = converter_write(value);
}
void MainWindow::on_spinBox_2_R_valueChanged(int arg1)
{
    ui->verticalSlider_2_R->setValue(arg1);
    write_buf[R2] = converter_write(arg1);
}
//Robô 3 - L
void MainWindow::on_verticalSlider_3_L_valueChanged(int value)
{
    ui->spinBox_3_L->setValue(value);
    write_buf[L3] = converter_write(value);
}
void MainWindow::on_spinBox_3_L_valueChanged(int arg1)
{
    ui->verticalSlider_3_L->setValue(arg1);
    write_buf[L3] = converter_write(arg1);
}
//Robô 3 - R
void MainWindow::on_verticalSlider_3_R_valueChanged(int value)
{
    ui->spinBox_3_R->setValue(value);
    write_buf[R3] = converter_write(value);
}
void MainWindow::on_spinBox_3_R_valueChanged(int arg1)
{
    ui->verticalSlider_3_R->setValue(arg1);
    write_buf[R3] = converter_write(arg1);
}
//Robô 4 - L
void MainWindow::on_verticalSlider_4_L_valueChanged(int value)
{
    ui->spinBox_4_L->setValue(value);
    write_buf[L4] = converter_write(value);
}
void MainWindow::on_spinBox_4_L_valueChanged(int arg1)
{
    ui->verticalSlider_4_L->setValue(arg1);
    write_buf[L4] = converter_write(arg1);
}
//Robô 4 - R
void MainWindow::on_verticalSlider_4_R_valueChanged(int value)
{
    ui->spinBox_4_R->setValue(value);
    write_buf[R4] = converter_write(value);
}
void MainWindow::on_spinBox_4_R_valueChanged(int arg1)
{
    ui->verticalSlider_4_R->setValue(arg1);
    write_buf[R4] = converter_write(arg1);
}
//Robô 5 - L
void MainWindow::on_verticalSlider_5_L_valueChanged(int value)
{
    ui->spinBox_5_L->setValue(value);
    write_buf[L5] = converter_write(value);
}
void MainWindow::on_spinBox_5_L_valueChanged(int arg1)
{
    ui->verticalSlider_5_L->setValue(arg1);
    write_buf[L5] = converter_write(arg1);
}
//Robô 5 - R
void MainWindow::on_verticalSlider_5_R_valueChanged(int value)
{
    ui->spinBox_5_R->setValue(value);
    write_buf[R5] = converter_write(value);
}
void MainWindow::on_spinBox_5_R_valueChanged(int arg1)
{
    ui->verticalSlider_5_R->setValue(arg1);
    write_buf[R5] = converter_write(arg1);
}

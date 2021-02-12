#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_legend.h>

/* MANDAR VELOCIDADE NULA SEM PRECISAR DO CAST SEMPRE */

#define C0 (char)0

/* DEFINES RELATIVOS À POSIÇÃO NO BUFFER DAS VELOCIDADES ENVIADAS */

#define L1 1
#define R1 2
#define L2 3
#define R2 4
#define L3 5
#define R3 6
#define L4 7
#define R4 8
#define L5 9
#define R5 10

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    unsigned char converter_write (int x);
    int converter_read (unsigned char c);
    void serial_config (void);
    void graph_config (void);
    void enviar_buffer (void);
    void ler_buffer (void);
private slots:

    void on_pushStartStop_clicked();

    void on_comboBox_currentIndexChanged(int index);

    void on_verticalSlider_1_L_valueChanged(int value);
    void on_spinBox_1_L_valueChanged(int arg1);

    void on_verticalSlider_1_R_valueChanged(int value);
    void on_spinBox_1_R_valueChanged(int arg1);

    void on_verticalSlider_2_L_valueChanged(int value);
    void on_spinBox_2_L_valueChanged(int arg1);

    void on_verticalSlider_2_R_valueChanged(int value);
    void on_spinBox_2_R_valueChanged(int arg1);

    void on_verticalSlider_3_L_valueChanged(int value);
    void on_spinBox_3_L_valueChanged(int arg1);

    void on_verticalSlider_3_R_valueChanged(int value);
    void on_spinBox_3_R_valueChanged(int arg1);

    void on_verticalSlider_4_L_valueChanged(int value);
    void on_spinBox_4_L_valueChanged(int arg1);

    void on_verticalSlider_4_R_valueChanged(int value);
    void on_spinBox_4_R_valueChanged(int arg1);

    void on_verticalSlider_5_L_valueChanged(int value);
    void on_spinBox_5_L_valueChanged(int arg1);

    void on_verticalSlider_5_R_valueChanged(int value);
    void on_spinBox_5_R_valueChanged(int arg1);
private:
    Ui::MainWindow *ui;
    bool flag_comunicacao = 0; // quando =1, indica que a comunicação está ativa
    int flag_robos = 0; // 0 = nenhum, 1-5 = robô 1-5
    unsigned char write_buf[11] = {C0, C0, C0, C0, C0, C0, C0, C0, C0, C0, C0};
    unsigned char read_buf[11] = {C0, C0, C0, C0, C0, C0, C0, C0, C0, C0, C0};
    QVector<double> xplot, yplot1, yplot2;
    int serial_port;
    QTimer *timer;
};
#endif // MAINWINDOW_H

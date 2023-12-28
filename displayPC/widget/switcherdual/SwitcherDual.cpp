#include "SwitcherDual.h"

SwitcherDual::SwitcherDual(QWidget *parent)
    : QWidget(parent),
      m_bChecked(false),
      m_background(Qt::gray)
{
    // 鼠标滑过光标形状 - 手型
    setCursor(Qt::PointingHandCursor);

    // 连接信号槽
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
}

// 绘制开关
void SwitcherDual::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::Antialiasing);

    QPainterPath path;
    QColor background;
    QColor thumbColor;
    qreal dOpacity;
    if (isEnabled()) { // 可用状态
        if (m_bChecked) { // 打开状态
            background = m_checkedColor;
            thumbColor = m_checkedColor;
            dOpacity = 0.600;
        } else { //关闭状态
            background = m_background;
            thumbColor = m_thumbColor;
            dOpacity = 0.800;
        }
    } else {  // 不可用状态
        background = m_background;
        dOpacity = 0.260;
        thumbColor = m_disabledColor;
    }
    // 绘制大椭圆
    painter.setBrush(background);
    painter.setOpacity(dOpacity);
    path.addRoundedRect(QRectF(m_nMargin, m_nMargin, width() - 2 * m_nMargin, height() - 2 * m_nMargin), m_radius, m_radius);
    painter.drawPath(path.simplified());

    // 绘制小椭圆
    painter.setBrush(thumbColor);
    painter.setOpacity(1.0);
    painter.drawEllipse(QRectF(m_nX - (m_nHeight / 2), m_nY - (m_nHeight / 2), height(), height()));
}

// 鼠标按下事件
void SwitcherDual::mousePressEvent(QMouseEvent *event)
{
    if (isEnabled()) {
        if (event->buttons() & Qt::LeftButton) {
            event->accept();
        } else {
            event->ignore();
        }
    }
}

// 鼠标释放事件 - 切换开关状态、发射toggled()信号
void SwitcherDual::mouseReleaseEvent(QMouseEvent *event)
{
    if (isEnabled()) {
        if ((event->type() == QMouseEvent::MouseButtonRelease) && (event->button() == Qt::LeftButton)) {
            event->accept();
            m_bChecked = !m_bChecked;
            emit toggled(m_bChecked);
            m_timer.start(3); // time of switcher sliding
        } else {
            event->ignore();
        }
    }
}

// 大小改变事件
void SwitcherDual::resizeEvent(QResizeEvent *event)
{
    m_nX = m_nHeight / 2;
    m_nY = m_nHeight / 2;
    QWidget::resizeEvent(event);
}

// 默认大小
QSize SwitcherDual::sizeHint() const
{
    return minimumSizeHint();
}

// 最小大小
QSize SwitcherDual::minimumSizeHint() const
{
    return QSize(2 * (m_nHeight + m_nMargin), m_nHeight + 2 * m_nMargin);
}

// 切换状态 - 滑动
void SwitcherDual::onTimeout()
{
    if (m_bChecked) {
        m_nX += 1;
        if (m_nX >= width() - m_nHeight)
            m_timer.stop();
    } else {
        m_nX -= 1;
        if (m_nX <= m_nHeight / 2)
            m_timer.stop();
    }
    update();
}

// 返回开关状态 - 打开：true 关闭：false
bool SwitcherDual::isToggled() const
{
    return m_bChecked;
}

// 设置开关状态
void SwitcherDual::setToggle(bool checked)
{
    m_bChecked = checked;
    m_timer.start(10);
}

// 设置背景颜色
void SwitcherDual::setBackgroundColor(QColor color)
{
    m_background = color;
}

// 设置选中颜色
void SwitcherDual::setCheckedColor(QColor color)
{
    m_checkedColor = color;
}

// 设置不可用颜色
void SwitcherDual::setDisbaledColor(QColor color)
{
    m_disabledColor = color;
}

bool SwitcherDual::isEnabled()
{
  return true;
}

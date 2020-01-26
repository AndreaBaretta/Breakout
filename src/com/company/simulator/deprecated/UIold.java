package com.company.simulator.deprecated;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.ImageObserver;
import java.awt.image.ImageProducer;
import java.io.File;
import java.io.IOException;

public class UIold {

    class DrawPanel extends JScrollPane {
        public DrawPanel(final JLabel label) {
            super(label);
        }

        @Override
        public void paintComponent(final Graphics g) {
            super.paintComponent(g);
            g.setColor(Color.BLUE);
            g.fillRect(10,10, 300,300);
        }
    }

    DrawPanel mainPanel;
    Image background = null;
    JLabel imagelabel;
    JPanel panel;

    public UIold() {
        try {
            background = ImageIO.read(new File("/home/andrea/Desktop/ftc-background.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        imagelabel = new JLabel(new ImageIcon(background));
        mainPanel = new DrawPanel(imagelabel);
        mainPanel.setPreferredSize(new Dimension(400,400));
    }

    public void createUI(final JFrame frame) {
        frame.add(mainPanel);
        //frame.add(ui.mainPanel, BorderLayout.CENTER);
    }
}

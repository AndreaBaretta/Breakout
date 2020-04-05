package com.company.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GL11;

import java.util.Arrays;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

public class Display {
    public long window;
    public int length;
    public int height;
    public double pixel_to_percentage_x;
    public double pixel_to_percentage_y;

    public Display(int length, int height) {
        this.length = length;
        this.height = height;
        pixel_to_percentage_x = 1d/((double)length/2d);
        pixel_to_percentage_y = 1d/((double)height/2d);
    }

    public void init() {
        if(!glfwInit()) {
            throw new IllegalStateException("Failed to initalize GLFW!");
        }
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(length, height, "test", 0, 0);
        //glfwGetPrimaryMonitor() Replace for full screen^
        if(window == 0) {
            throw new IllegalStateException("Failed to initalize Window!");
        }

        GLFWVidMode videoMode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        glfwSetWindowPos(window, (videoMode.width() - length) / 2, (videoMode.height() - height) / 2);
        GLFW.glfwMakeContextCurrent(window);
        GL.createCapabilities();
        GL11.glClearColor(1.0f, 0.0f, 0.0f, 0.0f);

        GLFW.glfwSwapInterval(1);

        glfwShowWindow(window);


    }

    public boolean isRunning() {
        return(!glfwWindowShouldClose(this.window));
    }

    public void update() {
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    public void terminate() {
        glfwTerminate();
    }

    public void setBackground(final double[] color) {
        glBegin(GL_QUADS);
        glColor3d(color[0], color[1], color[2]);
        glVertex2d(-1,-1);
        glVertex2d(1,-1);
        glVertex2d(1,1);
        glVertex2d(-1, 1);
        glEnd();
    }

    public void drawPolygon(final double[][] points, final double[] color, final int setting) {
//        GL11.glColor3d(color[0], color[1], color[2]);

        glBegin(setting);

        glColor3d(color[0], color[1], color[2]);
        for (double[] point : points) {
            glVertex2d(point[0]*pixel_to_percentage_x, point[1]*pixel_to_percentage_y);
        }

        glEnd();
    }

    public void drawRobotPixels(final double x, final double y, final double theta, final double width, final double height) {
        final double w = width/2;
        final double h = height/2;
//        final double sin = Math.sin(theta);
//        final double cos = Math.cos(theta);
//        drawPolygon(new double[][] {
//                /*{x-Math.cos(width/2),y-Math.sin(width/2)}, {x+Math.cos(width/2),y-Math.sin(width/2)}, {x+Math.cos(width/2),y+Math.sin(width/2)}, {x-Math.cos(width/2),y+Math.sin(width/2)}*/
//                {cos*-w + sin*-h + x, -sin*-w + cos*-h + y},
//                {cos*w + sin*-h + x, -sin*w + cos*-h + y},
//                {cos*w + sin*h + x, -sin*w + cos*h + y},
//                {cos*-w + sin*h + x, -sin*-w + cos*h + y}
//
//        }, new double[]{0,0,255}, GL_POLYGON);
        final double[][] transformation = new double[][]{
                CoordinateTransformations.toFieldCoordinates(new ArrayRealVector(new double[]{w, +h}), theta).toArray(),
                CoordinateTransformations.toFieldCoordinates(new ArrayRealVector(new double[]{-w, +h}), theta).toArray(),
                CoordinateTransformations.toFieldCoordinates(new ArrayRealVector(new double[]{-w, -h}), theta).toArray(),
                CoordinateTransformations.toFieldCoordinates(new ArrayRealVector(new double[]{w, -h}), theta).toArray()
        };
        drawPolygon(new double[][]{
                new double[] {x+transformation[0][0], y+transformation[0][1]},
                new double[] {x+transformation[1][0], y+transformation[1][1]},
                new double[] {x+transformation[2][0], y+transformation[2][1]},
                new double[] {x+transformation[3][0], y+transformation[3][1]}
        },new double[]{0,0,255}, GL_POLYGON);
//        drawPolygon();
        glBegin(GL_LINE_LOOP);
        glColor3d(255,0,0);
        glLineWidth(5);
        glVertex2d(x*pixel_to_percentage_x, y*pixel_to_percentage_y);
        final double[] transform = CoordinateTransformations.toFieldCoordinates(new ArrayRealVector(new double[]{w*pixel_to_percentage_y,0}), theta).toArray();

        glVertex2d(x*pixel_to_percentage_x+transform[0], y*pixel_to_percentage_y+transform[1]);
        glEnd();
        //GL_LINE_LOOP
//        glLoadIdentity();
    }

    public void drawRobot(final double x, final double y, final double theta, final double width, final double height, final double pxPerM) {
        final double x_px = x*pxPerM;
        final double y_px = y*pxPerM;
        final double width_px = width*pxPerM;
        final double height_px = height*pxPerM;
        drawRobotPixels(x_px, y_px, theta, width_px, height_px);
    }

    public void drawCirclePixels(double x, double y, double r, int num_segments, final double[] color)
    {
        glColor3d(color[0], color[1], color[2]);
        glBegin(GL_POLYGON);
        for(int ii = 0; ii < num_segments; ii++)
        {
            double theta = 2.0f * Math.PI * (double)ii / (double)num_segments;//get the current angle

            double cx = r * Math.cos(theta);//calculate the x component
            double cy = r * Math.sin(theta);//calculate the y component

            glVertex2d((x + cx)*pixel_to_percentage_x, (y + cy)*pixel_to_percentage_y);//output vertex

        }
        glEnd();
    }

    public void drawCircle(double x, double y, double r, int num_segments, final double[] color, final double metersToPixels) {
        final double x_px = x*metersToPixels;
        final double y_px = y*metersToPixels;
        final double r_px = r*metersToPixels;
        drawCirclePixels(x_px, y_px, r_px, num_segments, color);
    }
}

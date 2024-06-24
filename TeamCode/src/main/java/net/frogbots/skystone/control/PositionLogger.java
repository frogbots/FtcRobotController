package net.frogbots.skystone.control;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;
import java.io.Writer;

public class PositionLogger
{
    double[][] positions = new double[24000][2];

    int count = 0;

    public void add(double x, double y)
    {
        positions[count][0] = x;
        positions[count][1] = y;

        count++;
    }

    public void dump()
    {
        StringBuilder stringBuilder = new StringBuilder();

        for(int i = 0; i < count; i++)
        {
            stringBuilder.append(positions[i][0]).append(',').append(positions[i][1]).append("\n");
        }

        try (Writer writer = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream("/sdcard/odolog.txt"), "utf-8"))) {
            writer.write(stringBuilder.toString());
        } catch (Exception e)
        {
            e.printStackTrace();
        }
    }
}

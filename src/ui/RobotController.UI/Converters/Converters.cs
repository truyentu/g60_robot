using System.Globalization;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;

namespace RobotController.UI.Converters;

/// <summary>
/// Converts bool to Visibility
/// </summary>
public class BoolToVisibilityConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is bool b)
        {
            return b ? Visibility.Visible : Visibility.Collapsed;
        }
        return Visibility.Collapsed;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts bool to color (green for true, gray for false)
/// Supports ConverterParameter format: "TrueColor|FalseColor" (e.g., "#00FF00|#404040")
/// </summary>
public class BoolToColorConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool b = value is bool boolValue && boolValue;

        if (parameter is string paramStr && paramStr.Contains('|'))
        {
            var colors = paramStr.Split('|');
            var colorStr = b ? colors[0] : colors[1];

            try
            {
                var color = (Color)ColorConverter.ConvertFromString(colorStr);
                return new SolidColorBrush(color);
            }
            catch
            {
                return b ? Brushes.LimeGreen : Brushes.Gray;
            }
        }

        return b ? Brushes.LimeGreen : Brushes.Gray;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Inverts bool value
/// </summary>
public class InverseBoolConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is bool b)
        {
            return !b;
        }
        return false;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is bool b)
        {
            return !b;
        }
        return false;
    }
}

/// <summary>
/// Converts bool to text
/// Supports ConverterParameter format: "TrueText|FalseText" (e.g., "Connected|Disconnected")
/// </summary>
public class BoolToTextConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool b = value is bool boolValue && boolValue;

        if (parameter is string paramStr && paramStr.Contains('|'))
        {
            var texts = paramStr.Split('|');
            return b ? texts[0] : texts[1];
        }

        return b ? "True" : "False";
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

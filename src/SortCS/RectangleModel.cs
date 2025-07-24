using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SortCS;
public class RectangleModel
{
    public RectangleModel(object tag, RectangleF box)
    {
        Tag = tag;
        Box = box;
    }

    public object Tag { get; }
    public RectangleF Box { get; }
}

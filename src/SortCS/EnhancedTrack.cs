using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SortCS;
public record EnhancedTrack
{
    public int TrackId { get; set; }

    public int TotalMisses { get; set; }

    public int Misses { get; set; }

    public List<RectangleModel> History { get; set; }

    public TrackState State { get; set; }

    public RectangleF Prediction { get; set; }
    public DateTime UpdatedAt { get; set; } = DateTime.Now;
    public bool IsDirty { get; set; } = true;
    public object? Tag { get; set; }
    public bool IsConfirmed { get; set; }
}

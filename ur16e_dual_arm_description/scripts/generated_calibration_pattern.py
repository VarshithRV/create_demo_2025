from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4

def draw_square_diagram(pdf):
    width, height = A4
    outer_side_length = 0.85 * min(width, height)  # 85% of the shorter side
    margin = (min(width, height) - outer_side_length) / 2
    
    pdf.setLineWidth(5)
    pdf.setStrokeColorRGB(0,0,0)
    
    # Define points for squares
    outer_top_left = (margin, height - margin)
    outer_top_right = (margin + outer_side_length, height - margin)
    outer_bottom_left = (margin, height - margin - outer_side_length)
    outer_bottom_right = (margin + outer_side_length, height - margin - outer_side_length)
    
    inner_side_length = outer_side_length *0.85
    inner_top_left = outer_top_left
    inner_top_right = (outer_top_left[0]+inner_side_length, outer_top_left[1])
    inner_bottom_left = (outer_top_left[0]+inner_side_length, outer_top_left[1] - inner_side_length)
    inner_bottom_right = (outer_top_left, inner_bottom_left)
    
    # Draw the outer square
    pdf.rect(outer_bottom_left[0], outer_bottom_left[1], outer_side_length, outer_side_length)
    
    # Draw the inner square
    # pdf.rect(inner_bottom_left[0], inner_bottom_left[1], inner_side_length, inner_side_length)
    
    # Draw coordinate arrows
    pdf.setStrokeColorRGB(1, 0, 0)  # Red for X-axis
    pdf.setLineWidth(10)
    pdf.line(outer_bottom_left[0], outer_bottom_left[1], outer_bottom_left[0], outer_bottom_left[1] + outer_side_length * 0.2)  # Y-axis
    
    pdf.setStrokeColorRGB(0, 0.8, 0)  # Green for Y-axis
    pdf.line(outer_bottom_left[0], outer_bottom_left[1], outer_bottom_left[0] + (outer_side_length * 0.2), outer_bottom_left[1])  # X-axis
    pdf.setLineWidth(10)
    
    # Add labels
    pdf.setFillColorRGB(0, 0, 0)
    pdf.drawString(outer_top_left[0] - 20, outer_top_left[1] + 5, "1")
    pdf.drawString(outer_top_right[0] + 15, outer_top_right[1] + 5, "2")
    pdf.drawString(outer_bottom_right[0] + 15, outer_bottom_right[1] - 5, "3")
    pdf.drawString(outer_bottom_left[0] - 20, outer_bottom_left[1] - 5, "4")
#   pdf.drawString(inner_top_left[0] - 10, inner_top_left[1] + 5, "2")
#   pdf.drawString(inner_top_right[0] + 5, inner_top_right[1] + 5, "3")
#   pdf.drawString(inner_bottom_right[0] + 5, inner_bottom_right[1] - 5, "4")
    size_square = outer_side_length/2.834632756
    pdf.drawString(outer_bottom_left[0],outer_bottom_left[1] - 40, f"Side length = {size_square} mm")


    # Bullseye for outer square
    pdf.setStrokeColorRGB(0,0,0)
    pdf.setLineWidth(3)
    radius = outer_side_length*0.02
    for point in [outer_top_left, outer_top_right, outer_bottom_left, outer_bottom_right]:
    	pdf.circle(point[0],point[1],radius)

# Generate PDF
def create_pdf(output_path):
    pdf = canvas.Canvas(output_path, pagesize=A4)
    draw_square_diagram(pdf)
    pdf.save()

create_pdf("dual robot calibrator.pdf")


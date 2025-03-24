import plotly.express as px
import plotly.io as pio

# Create a sample Plotly figure
fig = px.scatter(x=[1, 2, 3], y=[3, 1, 6], title="Sample Scatter Plot")

# Convert figure to HTML without Plotly JS (we will add it manually)
html_content = pio.to_html(fig, full_html=False, include_plotlyjs=False)

# Define Plotly CDN script tag
plotly_js = '<script src="https://cdn.plot.ly/plotly-2.8.3.min.js"></script>'

# Wrap the content in MyST raw HTML block
myst_content = f"""```{{raw}} html
{plotly_js}
{html_content}"""